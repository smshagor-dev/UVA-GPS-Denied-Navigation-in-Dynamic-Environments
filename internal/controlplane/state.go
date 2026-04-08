// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

package controlplane

import (
	"fmt"
	"log"
	"sort"
	"strings"
	"sync"
	"time"
)

func asFloat64(value any) (float64, bool) {
	switch v := value.(type) {
	case float64:
		return v, true
	case float32:
		return float64(v), true
	case int:
		return float64(v), true
	case int32:
		return float64(v), true
	case int64:
		return float64(v), true
	default:
		return 0, false
	}
}

type FleetState struct {
	mu            sync.RWMutex
	drones        map[int]DroneTelemetry
	missions      map[string]MissionPlan
	commands      []CommandEnvelope
	events        []EventRecord
	clusterForm   map[string]string
	clusterLeader map[string]int
}

func NewFleetState() *FleetState {
	log.Printf("FleetState initialized")
	return &FleetState{
		drones:        make(map[int]DroneTelemetry),
		missions:      make(map[string]MissionPlan),
		clusterForm:   make(map[string]string),
		clusterLeader: make(map[string]int),
	}
}

func (s *FleetState) UpsertTelemetry(t DroneTelemetry) {
	log.Printf("FleetState.UpsertTelemetry drone=%d cluster=%s role=%s", t.DroneID, t.ClusterID, t.Role)
	s.mu.Lock()
	defer s.mu.Unlock()

	if t.Timestamp.IsZero() {
		t.Timestamp = time.Now().UTC()
	}
	if t.ClusterID == "" {
		t.ClusterID = fmt.Sprintf("cluster-%02d", ((t.DroneID-1)/20)+1)
	}
	if t.Role == "" {
		t.Role = "FOLLOWER"
	}
	if t.Connectivity == "" {
		t.Connectivity = "Mesh"
	}
	s.drones[t.DroneID] = t
	if t.Role == "LEADER" {
		s.clusterLeader[t.ClusterID] = t.DroneID
	}
	if _, ok := s.clusterForm[t.ClusterID]; !ok {
		s.clusterForm[t.ClusterID] = "DIAMOND"
	}
}

func (s *FleetState) Snapshot() FleetSnapshot {
	log.Printf("FleetState.Snapshot requested")
	s.mu.RLock()
	defer s.mu.RUnlock()

	drones := make([]DroneTelemetry, 0, len(s.drones))
	for _, drone := range s.drones {
		drones = append(drones, drone)
	}
	sort.Slice(drones, func(i, j int) bool { return drones[i].DroneID < drones[j].DroneID })

	clusterAgg := map[string]*ClusterState{}
	var leaderID int
	var cpuSum float64
	var gpuSum float64
	var total int
	var critical int
	for _, drone := range drones {
		total++
		cpuSum += drone.CPUTempC
		gpuSum += drone.GPULoadPct
		if drone.Role == "LEADER" && leaderID == 0 {
			leaderID = drone.DroneID
		}
		if drone.BatteryPct < 15 || drone.CPUTempC > 82 || !drone.Reachable || drone.LocalizationState == "lost" || drone.SyncConfidence < 0.35 {
			critical++
		}
		cluster := clusterAgg[drone.ClusterID]
		if cluster == nil {
			cluster = &ClusterState{
				ClusterID:    drone.ClusterID,
				LeaderID:     s.clusterLeader[drone.ClusterID],
				Formation:    s.clusterForm[drone.ClusterID],
				MissionState: drone.MissionState,
			}
			clusterAgg[drone.ClusterID] = cluster
		}
		cluster.DroneCount++
		cluster.AvgBattery += drone.BatteryPct
	}

	clusters := make([]ClusterState, 0, len(clusterAgg))
	for _, cluster := range clusterAgg {
		if cluster.DroneCount > 0 {
			cluster.AvgBattery /= float64(cluster.DroneCount)
		}
		clusters = append(clusters, *cluster)
	}
	sort.Slice(clusters, func(i, j int) bool { return clusters[i].ClusterID < clusters[j].ClusterID })

	var cpuAvg, gpuAvg float64
	if total > 0 {
		cpuAvg = cpuSum / float64(total)
		gpuAvg = gpuSum / float64(total)
	}

	return FleetSnapshot{
		Drones:         drones,
		LeaderID:       leaderID,
		AvgLatencyMS:   4.0,
		PacketLossPct:  0.5,
		CPUTempC:       cpuAvg,
		GPULoadPct:     gpuAvg,
		Timestamp:      time.Now().UTC(),
		Clusters:       clusters,
		CriticalAlerts: critical,
	}
}

func (s *FleetState) Health() HealthReport {
	log.Printf("FleetState.Health requested")
	snapshot := s.Snapshot()
	var online int
	var avgBattery float64
	var maxTemp float64
	for _, drone := range snapshot.Drones {
		if drone.Reachable {
			online++
		}
		avgBattery += drone.BatteryPct
		if drone.CPUTempC > maxTemp {
			maxTemp = drone.CPUTempC
		}
	}
	if len(snapshot.Drones) > 0 {
		avgBattery /= float64(len(snapshot.Drones))
	}
	return HealthReport{
		OnlineDrones:   online,
		TotalDrones:    len(snapshot.Drones),
		CriticalAlerts: snapshot.CriticalAlerts,
		AvgBatteryPct:  avgBattery,
		MaxCPUTempC:    maxTemp,
		UpdatedAt:      time.Now().UTC(),
	}
}

func (s *FleetState) RegisterMission(plan MissionPlan) {
	log.Printf("FleetState.RegisterMission mission=%s cluster=%s formation=%s", plan.MissionID, plan.ClusterID, plan.Formation)
	s.mu.Lock()
	defer s.mu.Unlock()
	if plan.CreatedAt.IsZero() {
		plan.CreatedAt = time.Now().UTC()
	}
	if plan.Status == "" {
		plan.Status = "scheduled"
	}
	s.missions[plan.MissionID] = plan
	if plan.ClusterID != "" && plan.Formation != "" {
		s.clusterForm[plan.ClusterID] = plan.Formation
	}
}

func (s *FleetState) Missions() []MissionPlan {
	log.Printf("FleetState.Missions requested")
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := make([]MissionPlan, 0, len(s.missions))
	for _, mission := range s.missions {
		out = append(out, mission)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].CreatedAt.Before(out[j].CreatedAt) })
	return out
}

func (s *FleetState) RecordCommand(cmd CommandEnvelope) {
	log.Printf("FleetState.RecordCommand id=%s action=%s", cmd.CommandID, cmd.Action)
	s.mu.Lock()
	defer s.mu.Unlock()
	s.commands = append(s.commands, cmd)
}

func (s *FleetState) Commands() []CommandEnvelope {
	log.Printf("FleetState.Commands requested")
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := make([]CommandEnvelope, len(s.commands))
	copy(out, s.commands)
	return out
}

func (s *FleetState) AddEvent(event EventRecord) {
	log.Printf("FleetState.AddEvent type=%s message=%s", event.Type, event.Message)
	s.mu.Lock()
	defer s.mu.Unlock()
	s.events = append(s.events, event)
	if len(s.events) > 512 {
		s.events = append([]EventRecord(nil), s.events[len(s.events)-512:]...)
	}
}

func (s *FleetState) Events() []EventRecord {
	log.Printf("FleetState.Events requested")
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := make([]EventRecord, len(s.events))
	copy(out, s.events)
	return out
}

func (s *FleetState) AddDrone(droneID int, clusterID string) DroneTelemetry {
	log.Printf("FleetState.AddDrone requested drone=%d cluster=%s", droneID, clusterID)
	s.mu.RLock()
	existingCount := len(s.drones)
	existing, exists := s.drones[droneID]
	s.mu.RUnlock()

	if droneID <= 0 {
		droneID = existingCount + 1
	}
	if exists {
		log.Printf("FleetState.AddDrone returning existing drone=%d", droneID)
		return existing
	}
	if clusterID == "" {
		clusterID = fmt.Sprintf("cluster-%02d", ((droneID-1)/20)+1)
	}
	role := "FOLLOWER"
	if _, hasLeader := s.clusterLeader[clusterID]; !hasLeader {
		role = "LEADER"
	}
	telemetry := DroneTelemetry{
		DroneID:                droneID,
		ClusterID:              clusterID,
		Role:                   role,
		Connectivity:           "Mesh",
		Reachable:              true,
		Position:               [3]float64{float64(droneID % 20), float64((droneID / 20) % 10), 8.0 + float64(droneID%3)},
		Velocity:               [3]float64{0.0, 0.0, 0.0},
		AttitudeRPY:            [3]float64{0.0, 0.0, 0.0},
		ThrustVector:           [3]float64{0.0, 0.0, 9.81},
		CommandedAltitudeM:     8.0,
		CommandedSpeedMPS:      3.0,
		DriftM:                 0.05,
		BatteryPct:             96.0,
		RSSIDBm:                -50.0,
		CPUTempC:               54.0,
		GPULoadPct:             38.0,
		MissionState:           "standby",
		LocalizationSource:     "vision-inertial",
		LocalizationState:      "nominal",
		LocalizationConfidence: 0.92,
		TDOAConfidence:         0.66,
		ConfidenceTrend:        0.03,
		RelocalizationCount:    0,
		VisibleAnchorCount:     5,
		OccupancyRatio:         0.14,
		SyncConfidence:         0.93,
		IMUCameraOffsetMS:      2.1,
		Timestamp:              time.Now().UTC(),
	}
	s.UpsertTelemetry(telemetry)
	log.Printf("FleetState.AddDrone created drone=%d cluster=%s role=%s", telemetry.DroneID, telemetry.ClusterID, telemetry.Role)
	return telemetry
}

func (s *FleetState) ApplyCommand(action string, clusterID string, targetIDs []int, payload map[string]any) int {
	log.Printf("FleetState.ApplyCommand action=%s cluster=%s targets=%v", action, clusterID, targetIDs)
	s.mu.Lock()
	defer s.mu.Unlock()

	action = strings.TrimSpace(strings.ToLower(action))
	if action == "" {
		return 0
	}

	targetSet := map[int]struct{}{}
	for _, id := range targetIDs {
		if id <= 0 {
			continue
		}
		targetSet[id] = struct{}{}
	}

	affected := 0
	for id, drone := range s.drones {
		if clusterID != "" && drone.ClusterID != clusterID {
			continue
		}
		if len(targetSet) > 0 {
			if _, ok := targetSet[id]; !ok {
				continue
			}
		}
		if altitude, ok := asFloat64(payload["altitude_m"]); ok && altitude > 0 {
			drone.CommandedAltitudeM = altitude
		}
		if speed, ok := asFloat64(payload["velocity_mps"]); ok && speed > 0 {
			drone.CommandedSpeedMPS = speed
		}

		switch action {
		case "formation":
			if shape, ok := payload["shape"].(string); ok {
				drone.MissionState = "formation-" + strings.ToLower(shape)
				s.clusterForm[drone.ClusterID] = shape
			} else {
				drone.MissionState = "formation-hold"
			}
		case "hold_position":
			drone.MissionState = "hold-position"
		case "return_home":
			drone.MissionState = "return-home"
		case "emergency_land":
			drone.MissionState = "emergency-land"
		case "fly":
			drone.MissionState = "fly"
		case "land":
			drone.MissionState = "land"
		case "election":
			drone.MissionState = "leader-election"
		case "remove_drone":
			delete(s.drones, id)
			affected++
			continue
		default:
			continue
		}
		drone.Timestamp = time.Now().UTC()
		s.drones[id] = drone
		affected++
	}

	return affected
}
