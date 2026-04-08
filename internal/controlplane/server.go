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
	"sort"
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
	s.seedDigitalTwin(5)
	go s.simulateFlightLoop()
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
			DroneID:                i,
			ClusterID:              clusterID,
			Role:                   role,
			Connectivity:           "Mesh",
			Reachable:              true,
			Position:               [3]float64{float64(i % 20), float64((i / 20) % 10), 8.0 + float64(i%3)},
			Velocity:               [3]float64{0.2, 0.1, 0.0},
			AttitudeRPY:            [3]float64{0.0, 0.0, 0.0},
			ThrustVector:           [3]float64{0.0, 0.0, 9.81},
			CommandedAltitudeM:     8.0 + float64(i%3)*0.5,
			CommandedSpeedMPS:      3.0,
			DriftM:                 0.08 + math.Mod(float64(i), 7)*0.01,
			BatteryPct:             92.0 - math.Mod(float64(i), 20),
			RSSIDBm:                -48.0 - math.Mod(float64(i), 8),
			CPUTempC:               58.0 + math.Mod(float64(i), 10),
			GPULoadPct:             42.0 + math.Mod(float64(i), 18),
			MissionState:           "patrol",
			LocalizationSource:     "vision-depth-fused",
			LocalizationState:      "nominal",
			LocalizationConfidence: 0.90 - math.Mod(float64(i), 5)*0.04,
			TDOAConfidence:         0.68 - math.Mod(float64(i), 4)*0.05,
			ConfidenceTrend:        0.02 - math.Mod(float64(i), 3)*0.01,
			RelocalizationCount:    (i - 1) / 22,
			VisibleAnchorCount:     5 - (i % 3),
			OccupancyRatio:         0.10 + math.Mod(float64(i), 6)*0.02,
			SyncConfidence:         0.96 - math.Mod(float64(i), 5)*0.05,
			IMUCameraOffsetMS:      1.2 + math.Mod(float64(i), 4)*0.7,
			Timestamp:              time.Now().UTC(),
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

func (s *Server) simulateFlightLoop() {
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()
	started := time.Now()
	lastTick := started
	electionEndTime := time.Time{}

	for range ticker.C {
		s.state.mu.Lock()
		now := time.Now()
		elapsed := now.Sub(started).Seconds()
		dt := now.Sub(lastTick).Seconds()
		lastTick = now
		if dt <= 0 {
			dt = 0.05
		}

		leaderID := 1
		electionActive := false
		for _, drone := range s.state.drones {
			if drone.Role == "LEADER" {
				leaderID = drone.DroneID
			}
			if drone.MissionState == "leader-election" {
				electionActive = true
			}
		}

		if electionActive {
			if electionEndTime.IsZero() {
				electionEndTime = time.Now().Add(1500 * time.Millisecond)
			}
			if time.Now().After(electionEndTime) {
				bestScore := -1.0
				bestID := leaderID
				for id, d := range s.state.drones {
					// MCSS Score calc
					score := (d.BatteryPct/100.0)*0.4 + ((d.RSSIDBm+90.0)/60.0)*0.3 + d.LocalizationConfidence*0.3
					// Add some random noise to make elections competitive in the simulation
					score += (math.Sin(elapsed*float64(id)) * 0.05)

					if score > bestScore {
						bestScore = score
						bestID = id
					}
				}
				leaderID = bestID
				for id, d := range s.state.drones {
					if id == leaderID {
						d.Role = "LEADER"
					} else {
						d.Role = "FOLLOWER"
					}
					d.MissionState = "formation-hold" // return to normal flight
					s.state.drones[id] = d
				}
				electionEndTime = time.Time{}
			}
		}

		leaderPos, leaderVel := leaderFigureEight(elapsed)
		followerIDs := make([]int, 0, len(s.state.drones))
		for id := range s.state.drones {
			if id != leaderID {
				followerIDs = append(followerIDs, id)
			}
		}
		sort.Ints(followerIDs)
		followerSlots := make(map[int]int, len(followerIDs))
		for idx, id := range followerIDs {
			followerSlots[id] = idx + 1
		}

		for id, drone := range s.state.drones {
			drone.BatteryPct = math.Max(15.0, 100.0-elapsed*(0.015+float64(id)*0.005))
			drone.CPUTempC = 50.0 + 10.0*math.Sin(elapsed*0.1+float64(id))

			targetPos, targetVel, maxSpeed, maxAccel := flightTargetForDrone(
				drone,
				leaderID,
				followerSlots[id],
				leaderPos,
				leaderVel,
			)
			drone.Position, drone.Velocity, drone.ThrustVector, drone.AttitudeRPY = integrateDroneKinematics(
				drone.Position,
				drone.Velocity,
				targetPos,
				targetVel,
				dt,
				maxSpeed,
				maxAccel,
			)
			if drone.Position[2] <= 0.02 && (drone.MissionState == "land" || drone.MissionState == "emergency-land") {
				drone.Position[2] = 0
				drone.Velocity = [3]float64{}
			}

			drone.Timestamp = now.UTC()
			s.state.drones[id] = drone
		}
		s.state.mu.Unlock()
	}
}

func leaderFigureEight(elapsed float64) ([3]float64, [3]float64) {
	omega := 0.22
	x := 14.0 * math.Sin(omega*elapsed)
	y := 7.5 * math.Sin(2.0*omega*elapsed)
	z := 5.0 + 0.8*math.Sin(0.35*elapsed)
	vx := 14.0 * omega * math.Cos(omega*elapsed)
	vy := 15.0 * omega * math.Cos(2.0*omega*elapsed)
	vz := 0.28 * math.Cos(0.35*elapsed)
	return [3]float64{x, y, z}, [3]float64{vx, vy, vz}
}

func flightTargetForDrone(drone DroneTelemetry, leaderID int, slot int, leaderPos [3]float64, leaderVel [3]float64) ([3]float64, [3]float64, float64, float64) {
	targetAltitude := drone.CommandedAltitudeM
	if targetAltitude < 3.0 {
		targetAltitude = 5.0
	}
	maxSpeed := drone.CommandedSpeedMPS
	if maxSpeed < 0.8 {
		maxSpeed = 2.5
	}
	maxAccel := math.Max(1.2, maxSpeed*1.1)

	switch drone.MissionState {
	case "emergency-land":
		return [3]float64{drone.Position[0], drone.Position[1], 0}, [3]float64{}, math.Min(maxSpeed, 1.8), maxAccel * 1.4
	case "land":
		return [3]float64{drone.Position[0], drone.Position[1], 0}, [3]float64{}, math.Min(maxSpeed, 1.2), maxAccel
	case "return-home":
		return [3]float64{0, 0, targetAltitude}, [3]float64{}, maxSpeed, maxAccel
	case "hold-position", "leader-election":
		return drone.Position, [3]float64{}, math.Min(maxSpeed, 1.0), maxAccel
	}

	if drone.DroneID == leaderID {
		return leaderPos, leaderVel, math.Max(maxSpeed, 3.2), math.Max(maxAccel, 2.6)
	}

	if slot <= 0 {
		slot = 1
	}
	spacing := 2.8
	targetPos := leaderPos
	targetVel := leaderVel

	switch {
	case strings.HasPrefix(drone.MissionState, "formation-line"):
		targetPos[0] += float64(slot) * spacing
		targetPos[2] = targetAltitude
	case strings.HasPrefix(drone.MissionState, "formation-vee"):
		rank := float64((slot + 1) / 2)
		side := 1.0
		if slot%2 == 0 {
			side = -1.0
		}
		targetPos[0] -= rank * spacing * 1.2
		targetPos[1] += side * rank * spacing
		targetPos[2] = targetAltitude
	default:
		targetPos = followerDiamondSlot(slot, leaderPos, targetAltitude, spacing)
	}

	if drone.MissionState == "fly" && drone.Position[2] < 0.5 {
		targetPos[0] = drone.Position[0]
		targetPos[1] = drone.Position[1]
		targetVel[0] = 0
		targetVel[1] = 0
	}

	return targetPos, targetVel, maxSpeed, maxAccel
}

func followerDiamondSlot(slot int, leaderPos [3]float64, altitude float64, spacing float64) [3]float64 {
	offsets := [][3]float64{
		{-spacing, 0, 0},
		{-2 * spacing, spacing, 0},
		{-2 * spacing, -spacing, 0},
		{-3 * spacing, 0, 0},
		{-4 * spacing, 2 * spacing, 0},
		{-4 * spacing, -2 * spacing, 0},
	}
	idx := slot - 1
	if idx < len(offsets) {
		return [3]float64{
			leaderPos[0] + offsets[idx][0],
			leaderPos[1] + offsets[idx][1],
			altitude,
		}
	}
	ring := float64(idx-len(offsets)+1) / 2.0
	phase := float64(idx) * math.Pi / 3.0
	return [3]float64{
		leaderPos[0] - (3.0+ring)*spacing + math.Cos(phase)*spacing,
		leaderPos[1] + math.Sin(phase)*spacing*(1.0+0.2*ring),
		altitude,
	}
}

func integrateDroneKinematics(position [3]float64, velocity [3]float64, targetPos [3]float64, targetVel [3]float64, dt float64, maxSpeed float64, maxAccel float64) ([3]float64, [3]float64, [3]float64, [3]float64) {
	accel, thrustVector, attitude := computeGuidanceCommand(position, velocity, targetPos, targetVel, maxSpeed, maxAccel)
	velocity = vecAdd(velocity, vecScale(accel, dt))
	velocity = clampMagnitude(velocity, maxSpeed)
	position = vecAdd(position, vecScale(velocity, dt))

	if vecNorm(vecSub(targetPos, position)) < 0.08 && vecNorm(velocity) < 0.12 {
		position = targetPos
		velocity = targetVel
		thrustVector = [3]float64{0.0, 0.0, 9.81}
		attitude = guidanceAttitude(targetVel, thrustVector)
	}
	if position[2] < 0 {
		position[2] = 0
		if velocity[2] < 0 {
			velocity[2] = 0
		}
	}
	return position, velocity, thrustVector, attitude
}

func computeGuidanceCommand(position [3]float64, velocity [3]float64, targetPos [3]float64, targetVel [3]float64, maxSpeed float64, maxAccel float64) ([3]float64, [3]float64, [3]float64) {
	posError := vecSub(targetPos, position)
	desiredVel := vecAdd(targetVel, vecScale(posError, 0.85))
	desiredVel = clampMagnitude(desiredVel, maxSpeed)

	accel := vecScale(vecSub(desiredVel, velocity), 1.8)
	accel = clampMagnitude(accel, maxAccel)
	thrustVector := [3]float64{accel[0], accel[1], 9.81 + accel[2]}
	attitude := guidanceAttitude(desiredVel, thrustVector)
	return accel, thrustVector, attitude
}

func guidanceAttitude(referenceVel [3]float64, thrustVector [3]float64) [3]float64 {
	const gravity = 9.81
	pitch := math.Atan2(thrustVector[0], gravity)
	roll := math.Atan2(-thrustVector[1], gravity)
	yaw := 0.0
	if math.Abs(referenceVel[0]) > 1e-6 || math.Abs(referenceVel[1]) > 1e-6 {
		yaw = math.Atan2(referenceVel[1], referenceVel[0])
	}
	return [3]float64{roll, pitch, yaw}
}

func vecAdd(a [3]float64, b [3]float64) [3]float64 {
	return [3]float64{a[0] + b[0], a[1] + b[1], a[2] + b[2]}
}

func vecSub(a [3]float64, b [3]float64) [3]float64 {
	return [3]float64{a[0] - b[0], a[1] - b[1], a[2] - b[2]}
}

func vecScale(v [3]float64, scalar float64) [3]float64 {
	return [3]float64{v[0] * scalar, v[1] * scalar, v[2] * scalar}
}

func vecNorm(v [3]float64) float64 {
	return math.Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
}

func clampMagnitude(v [3]float64, max float64) [3]float64 {
	if max <= 0 {
		return [3]float64{}
	}
	norm := vecNorm(v)
	if norm == 0 || norm <= max {
		return v
	}
	return vecScale(v, max/norm)
}
