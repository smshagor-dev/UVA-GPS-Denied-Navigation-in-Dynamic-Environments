// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

package controlplane

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"math"
	"net/http"
	"strconv"
	"strings"
	"time"
)

type Server struct {
	addr  string
	state *FleetState
	http  *http.Server
}

func NewServer(addr string) *Server {
	state := NewFleetState()
	s := &Server{
		addr:  addr,
		state: state,
	}
	mux := http.NewServeMux()
	mux.HandleFunc("/api/v1/telemetry", s.withRecovery(s.handleTelemetry))
	mux.HandleFunc("/api/v1/fleet", s.withRecovery(s.handleFleet))
	mux.HandleFunc("/api/v1/commands", s.withRecovery(s.handleCommands))
	mux.HandleFunc("/api/v1/missions", s.withRecovery(s.handleMissions))
	mux.HandleFunc("/api/v1/health", s.withRecovery(s.handleHealth))
	mux.HandleFunc("/api/v1/events", s.withRecovery(s.handleEvents))
	mux.HandleFunc("/api/v1/discovery", s.withRecovery(s.handleDiscovery))
	s.http = &http.Server{
		Addr:              addr,
		Handler:           mux,
		ReadHeaderTimeout: 3 * time.Second,
		ReadTimeout:       5 * time.Second,
		WriteTimeout:      5 * time.Second,
		IdleTimeout:       30 * time.Second,
	}
	s.seedDigitalTwin(120)
	return s
}

func (s *Server) Start() error {
	log.Printf("go control-plane listening on %s", s.addr)
	err := s.http.ListenAndServe()
	if errors.Is(err, http.ErrServerClosed) {
		return nil
	}
	return err
}

func (s *Server) Shutdown(ctx context.Context) error {
	return s.http.Shutdown(ctx)
}

func (s *Server) seedDigitalTwin(n int) {
	for i := 1; i <= n; i++ {
		clusterID := fmt.Sprintf("cluster-%02d", ((i-1)/20)+1)
		role := "FOLLOWER"
		if (i-1)%20 == 0 {
			role = "LEADER"
		}
		s.state.UpsertTelemetry(DroneTelemetry{
			DroneID:      i,
			ClusterID:    clusterID,
			Role:         role,
			Connectivity: "Mesh",
			Reachable:    true,
			Position:     [3]float64{float64(i % 20), float64((i / 20) % 10), 8.0 + float64(i%3)},
			Velocity:     [3]float64{0.2, 0.1, 0.0},
			DriftM:       0.08 + math.Mod(float64(i), 7)*0.01,
			BatteryPct:   92.0 - math.Mod(float64(i), 20),
			RSSIDBm:      -48.0 - math.Mod(float64(i), 8),
			CPUTempC:     58.0 + math.Mod(float64(i), 10),
			GPULoadPct:   42.0 + math.Mod(float64(i), 18),
			MissionState: "patrol",
			Timestamp:    time.Now().UTC(),
		})
	}
	s.state.RegisterMission(MissionPlan{
		MissionID: "mission-bootstrap",
		Name:      "Bootstrap Patrol",
		Formation: "DIAMOND",
		ClusterID: "cluster-01",
		Target:    [3]float64{50, 10, 12},
	})
	s.state.AddEvent(EventRecord{
		Type:      "bootstrap",
		Message:   "digital twin seeded",
		Timestamp: time.Now().UTC(),
	})
}

func (s *Server) handleTelemetry(w http.ResponseWriter, r *http.Request) {
	log.Printf("handleTelemetry method=%s remote=%s", r.Method, r.RemoteAddr)
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "method not allowed"})
		return
	}
	var telemetry DroneTelemetry
	if err := decodeJSONBody(r, &telemetry); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"error": err.Error()})
		return
	}
	if telemetry.DroneID <= 0 {
		writeJSON(w, http.StatusBadRequest, map[string]any{"error": "invalid drone_id"})
		return
	}
	s.state.UpsertTelemetry(telemetry)
	s.state.AddEvent(EventRecord{
		Type:      "telemetry",
		Message:   fmt.Sprintf("drone %d telemetry ingested", telemetry.DroneID),
		Timestamp: time.Now().UTC(),
		Data:      map[string]any{"cluster_id": telemetry.ClusterID},
	})
	writeJSON(w, http.StatusAccepted, map[string]any{"message": "telemetry accepted"})
}

func (s *Server) handleFleet(w http.ResponseWriter, r *http.Request) {
	log.Printf("handleFleet method=%s remote=%s", r.Method, r.RemoteAddr)
	if r.Method != http.MethodGet {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "method not allowed"})
		return
	}
	snapshot := s.state.Snapshot()
	clusterFilter := r.URL.Query().Get("cluster_id")
	if clusterFilter != "" {
		filtered := snapshot.Drones[:0]
		for _, drone := range snapshot.Drones {
			if drone.ClusterID == clusterFilter {
				filtered = append(filtered, drone)
			}
		}
		snapshot.Drones = filtered
	}
	writeJSON(w, http.StatusOK, snapshot)
}

func (s *Server) handleCommands(w http.ResponseWriter, r *http.Request) {
	log.Printf("handleCommands method=%s remote=%s", r.Method, r.RemoteAddr)
	if r.Method != http.MethodPost {
		writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "method not allowed"})
		return
	}
	var payload struct {
		Action  string         `json:"action"`
		Payload map[string]any `json:"payload"`
	}
	if err := decodeJSONBody(r, &payload); err != nil {
		writeJSON(w, http.StatusBadRequest, map[string]any{"error": err.Error()})
		return
	}
	payload.Action = strings.TrimSpace(strings.ToLower(payload.Action))
	if payload.Action == "" {
		writeJSON(w, http.StatusBadRequest, map[string]any{"error": "action is required"})
		return
	}
	if payload.Payload == nil {
		payload.Payload = map[string]any{}
	}
	cmd := CommandEnvelope{
		CommandID: fmt.Sprintf("cmd-%d", time.Now().UnixNano()),
		Action:    payload.Action,
		Payload:   payload.Payload,
		IssuedBy:  "dashboard",
		IssuedAt:  time.Now().UTC(),
	}
	if clusterID, ok := payload.Payload["cluster_id"].(string); ok {
		cmd.ClusterID = strings.TrimSpace(clusterID)
	}
	if rawTargets, ok := payload.Payload["target_ids"].([]any); ok {
		for _, raw := range rawTargets {
			if v, ok := raw.(float64); ok {
				id := int(v)
				if id > 0 {
					cmd.TargetIDs = append(cmd.TargetIDs, id)
				}
			}
		}
	}
	s.state.RecordCommand(cmd)
	s.state.AddEvent(EventRecord{
		Type:      "command",
		Message:   fmt.Sprintf("%s command queued", strings.ToUpper(payload.Action)),
		Timestamp: time.Now().UTC(),
		Data:      map[string]any{"command_id": cmd.CommandID},
	})
	if shape, ok := payload.Payload["shape"].(string); ok && cmd.ClusterID != "" {
		s.state.RegisterMission(MissionPlan{
			MissionID: fmt.Sprintf("mission-%d", time.Now().UnixNano()),
			Name:      "Formation Update",
			Formation: shape,
			ClusterID: cmd.ClusterID,
			Status:    "running",
		})
	}
	if payload.Action == "add_drone" {
		droneID := 0
		if raw, ok := payload.Payload["drone_id"]; ok {
			switch v := raw.(type) {
			case float64:
				droneID = int(v)
			case int:
				droneID = v
			}
		}
		clusterID, _ := payload.Payload["cluster_id"].(string)
		telemetry := s.state.AddDrone(droneID, clusterID)
		writeJSON(w, http.StatusAccepted, map[string]any{
			"message":    fmt.Sprintf("drone %d added to swarm", telemetry.DroneID),
			"command_id": cmd.CommandID,
		})
		return
	}
	affected := s.state.ApplyCommand(payload.Action, cmd.ClusterID, cmd.TargetIDs, payload.Payload)
	writeJSON(w, http.StatusAccepted, map[string]any{
		"message":    fmt.Sprintf("%s accepted by command fan-out service for %d drone(s)", payload.Action, affected),
		"command_id": cmd.CommandID,
	})
}

func (s *Server) handleMissions(w http.ResponseWriter, r *http.Request) {
	log.Printf("handleMissions method=%s remote=%s", r.Method, r.RemoteAddr)
	switch r.Method {
	case http.MethodGet:
		writeJSON(w, http.StatusOK, map[string]any{"missions": s.state.Missions()})
	case http.MethodPost:
		var plan MissionPlan
		if err := decodeJSONBody(r, &plan); err != nil {
			writeJSON(w, http.StatusBadRequest, map[string]any{"error": err.Error()})
			return
		}
		if plan.MissionID == "" {
			plan.MissionID = fmt.Sprintf("mission-%d", time.Now().UnixNano())
		}
		s.state.RegisterMission(plan)
		s.state.AddEvent(EventRecord{
			Type:      "mission",
			Message:   fmt.Sprintf("mission %s scheduled", plan.MissionID),
			Timestamp: time.Now().UTC(),
		})
		writeJSON(w, http.StatusCreated, plan)
	default:
		writeJSON(w, http.StatusMethodNotAllowed, map[string]any{"error": "method not allowed"})
	}
}

func (s *Server) handleHealth(w http.ResponseWriter, r *http.Request) {
	log.Printf("handleHealth method=%s remote=%s", r.Method, r.RemoteAddr)
	writeJSON(w, http.StatusOK, s.state.Health())
}

func (s *Server) handleEvents(w http.ResponseWriter, r *http.Request) {
	log.Printf("handleEvents method=%s remote=%s", r.Method, r.RemoteAddr)
	limit := 50
	if raw := r.URL.Query().Get("limit"); raw != "" {
		if parsed, err := strconv.Atoi(raw); err == nil && parsed > 0 {
			limit = parsed
		}
	}
	events := s.state.Events()
	if len(events) > limit {
		events = events[len(events)-limit:]
	}
	writeJSON(w, http.StatusOK, map[string]any{"events": events})
}

func (s *Server) handleDiscovery(w http.ResponseWriter, r *http.Request) {
	log.Printf("handleDiscovery method=%s remote=%s", r.Method, r.RemoteAddr)
	snapshot := s.state.Snapshot()
	writeJSON(w, http.StatusOK, map[string]any{
		"clusters": snapshot.Clusters,
		"services": []string{
			"telemetry-gateway",
			"swarm-registry",
			"mission-scheduler",
			"formation-orchestrator",
			"health-monitor",
			"command-fanout",
			"dashboard-gateway",
			"log-pipeline",
			"digital-twin-cache",
		},
	})
}

func writeJSON(w http.ResponseWriter, status int, value any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	if err := json.NewEncoder(w).Encode(value); err != nil {
		log.Printf("writeJSON failed: %v", err)
	}
}

func decodeJSONBody(r *http.Request, target any) error {
	defer r.Body.Close()
	r.Body = io.NopCloser(io.LimitReader(r.Body, 1<<20))
	decoder := json.NewDecoder(r.Body)
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(target); err != nil {
		if errors.Is(err, io.EOF) {
			return errors.New("request body is empty")
		}
		return err
	}
	if decoder.More() {
		return errors.New("request body must contain a single JSON object")
	}
	return nil
}

func (s *Server) withRecovery(next http.HandlerFunc) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		started := time.Now()
		defer func() {
			if rec := recover(); rec != nil {
				log.Printf("panic recovered on %s %s: %v", r.Method, r.URL.Path, rec)
				writeJSON(w, http.StatusInternalServerError, map[string]any{"error": "internal server error"})
			}
			log.Printf("request complete method=%s path=%s remote=%s duration=%s",
				r.Method, r.URL.Path, r.RemoteAddr, time.Since(started))
		}()
		next(w, r)
	}
}
