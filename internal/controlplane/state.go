// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

package controlplane

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"sort"
	"strings"
	"sync"
	"time"
)

const (
	maxLidarPoints2D         = 256
	maxTDOAAnchors           = 32
	maxReplayConfidenceCount = 256
	maxPreviewURLLength      = 512
	maxFrameRefLength        = 256
	maxResolutionLength      = 32
	maxFileNameLength        = 256
	maxAnchorIDLength        = 32
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

func movementStep(payload map[string]any) float64 {
	if value, ok := asFloat64(payload["step_m"]); ok && value > 0 {
		return value
	}
	return 1.5
}

type FleetState struct {
	mu            sync.RWMutex
	drones        map[int]DroneTelemetry
	missions      map[string]MissionPlan
	commands      []CommandEnvelope
	approvals     map[string]PendingApproval
	events        []EventRecord
	clusterForm   map[string]string
	clusterLeader map[string]int
	lastAuditHash string
	backendMode   string
	staleAfter    time.Duration
	simulated     bool
}

func NewFleetState(backendMode string, simulationEnabled bool, staleAfter time.Duration) *FleetState {
	if strings.TrimSpace(backendMode) == "" {
		backendMode = "simulation"
	}
	if staleAfter <= 0 {
		staleAfter = 5 * time.Second
	}
	log.Printf("FleetState initialized backend_mode=%s simulation_enabled=%t stale_after=%s", backendMode, simulationEnabled, staleAfter)
	return &FleetState{
		drones:        make(map[int]DroneTelemetry),
		missions:      make(map[string]MissionPlan),
		approvals:     make(map[string]PendingApproval),
		clusterForm:   make(map[string]string),
		clusterLeader: make(map[string]int),
		backendMode:   strings.TrimSpace(strings.ToLower(backendMode)),
		staleAfter:    staleAfter,
		simulated:     simulationEnabled,
	}
}

func normalizeDroneSource(source string) string {
	switch strings.TrimSpace(strings.ToLower(source)) {
	case "real", "simulation", "playback", "unavailable":
		return strings.TrimSpace(strings.ToLower(source))
	default:
		return "unavailable"
	}
}

func normalizeSensorStatus(status string) string {
	switch strings.TrimSpace(strings.ToLower(status)) {
	case "live", "ok", "ready":
		return "live"
	case "playback":
		return "playback"
	case "simulation":
		return "simulation"
	case "stale":
		return "stale"
	case "fault", "error":
		return "fault"
	case "unavailable", "disconnected":
		return "unavailable"
	default:
		return "unavailable"
	}
}

func trimString(value string, maxLen int) string {
	value = strings.TrimSpace(value)
	if maxLen > 0 && len(value) > maxLen {
		return value[:maxLen]
	}
	return value
}

func sanitizeCameraTelemetry(camera CameraTelemetry) CameraTelemetry {
	camera.Status = normalizeSensorStatus(camera.Status)
	camera.Source = normalizeDroneSource(camera.Source)
	camera.Resolution = trimString(camera.Resolution, maxResolutionLength)
	camera.PreviewURL = trimString(camera.PreviewURL, maxPreviewURLLength)
	camera.LatestFrameRef = trimString(camera.LatestFrameRef, maxFrameRefLength)
	if camera.FPS < 0 || math.IsNaN(camera.FPS) || math.IsInf(camera.FPS, 0) {
		camera.FPS = 0
	}
	if camera.FrameAgeMS < 0 || math.IsNaN(camera.FrameAgeMS) || math.IsInf(camera.FrameAgeMS, 0) {
		camera.FrameAgeMS = 0
	}
	if camera.DroppedFrames < 0 {
		camera.DroppedFrames = 0
	}
	return camera
}

func sanitizeIMUTelemetry(imu IMUTelemetry) IMUTelemetry {
	imu.Status = normalizeSensorStatus(imu.Status)
	imu.Source = normalizeDroneSource(imu.Source)
	imu.Health = trimString(imu.Health, 48)
	if imu.SampleRateHz < 0 || math.IsNaN(imu.SampleRateHz) || math.IsInf(imu.SampleRateHz, 0) {
		imu.SampleRateHz = 0
	}
	if imu.LastSampleAgeMS < 0 || math.IsNaN(imu.LastSampleAgeMS) || math.IsInf(imu.LastSampleAgeMS, 0) {
		imu.LastSampleAgeMS = 0
	}
	return imu
}

func sanitizeLiDARTelemetry(lidar LiDARTelemetry) LiDARTelemetry {
	lidar.Status = normalizeSensorStatus(lidar.Status)
	lidar.Source = normalizeDroneSource(lidar.Source)
	if lidar.PacketRateHz < 0 || math.IsNaN(lidar.PacketRateHz) || math.IsInf(lidar.PacketRateHz, 0) {
		lidar.PacketRateHz = 0
	}
	if lidar.ScanAgeMS < 0 || math.IsNaN(lidar.ScanAgeMS) || math.IsInf(lidar.ScanAgeMS, 0) {
		lidar.ScanAgeMS = 0
	}
	if lidar.MinRangeM < 0 || math.IsNaN(lidar.MinRangeM) || math.IsInf(lidar.MinRangeM, 0) {
		lidar.MinRangeM = 0
	}
	if lidar.MaxRangeM < 0 || math.IsNaN(lidar.MaxRangeM) || math.IsInf(lidar.MaxRangeM, 0) {
		lidar.MaxRangeM = 0
	}
	if len(lidar.Points2D) > maxLidarPoints2D {
		lidar.Points2D = append([]LidarPoint2D(nil), lidar.Points2D[:maxLidarPoints2D]...)
	}
	if lidar.PointCount <= 0 || lidar.PointCount > len(lidar.Points2D) {
		lidar.PointCount = len(lidar.Points2D)
	}
	return lidar
}

func sanitizeTDOATelemetry(tdoa TDOATelemetry) TDOATelemetry {
	tdoa.Status = normalizeSensorStatus(tdoa.Status)
	tdoa.Source = normalizeDroneSource(tdoa.Source)
	tdoa.CalibrationWarning = trimString(tdoa.CalibrationWarning, 256)
	if len(tdoa.Anchors) > maxTDOAAnchors {
		tdoa.Anchors = append([]TDOAAnchorTelemetry(nil), tdoa.Anchors[:maxTDOAAnchors]...)
	}
	for idx := range tdoa.Anchors {
		tdoa.Anchors[idx].ID = trimString(tdoa.Anchors[idx].ID, maxAnchorIDLength)
		if tdoa.Anchors[idx].LastSeenMS < 0 || math.IsNaN(tdoa.Anchors[idx].LastSeenMS) || math.IsInf(tdoa.Anchors[idx].LastSeenMS, 0) {
			tdoa.Anchors[idx].LastSeenMS = 0
		}
	}
	if tdoa.VisibleAnchorCount <= 0 || tdoa.VisibleAnchorCount > len(tdoa.Anchors) {
		count := 0
		for _, anchor := range tdoa.Anchors {
			if anchor.Visible {
				count++
			}
		}
		tdoa.VisibleAnchorCount = count
	}
	return tdoa
}

func sanitizeReplayTelemetry(replay ReplayTelemetry) ReplayTelemetry {
	replay.Status = normalizeSensorStatus(replay.Status)
	replay.Source = normalizeDroneSource(replay.Source)
	replay.FileName = trimString(replay.FileName, maxFileNameLength)
	if replay.Progress < 0 || math.IsNaN(replay.Progress) || math.IsInf(replay.Progress, 0) {
		replay.Progress = 0
	}
	if replay.Progress > 1 {
		replay.Progress = 1
	}
	if replay.CurrentTime < 0 || math.IsNaN(replay.CurrentTime) || math.IsInf(replay.CurrentTime, 0) {
		replay.CurrentTime = 0
	}
	if len(replay.ConfidenceSeries) > maxReplayConfidenceCount {
		replay.ConfidenceSeries = append([]float64(nil), replay.ConfidenceSeries[:maxReplayConfidenceCount]...)
	}
	for idx, value := range replay.ConfidenceSeries {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			replay.ConfidenceSeries[idx] = 0
			continue
		}
		if value < 0 {
			replay.ConfidenceSeries[idx] = 0
		} else if value > 1 {
			replay.ConfidenceSeries[idx] = 1
		}
	}
	return replay
}

func markDroneStale(drone *DroneTelemetry, staleAfter time.Duration, now time.Time) {
	drone.Stale = false
	if drone.Timestamp.IsZero() {
		return
	}
	if staleAfter > 0 && now.Sub(drone.Timestamp) > staleAfter {
		drone.Stale = true
		drone.Reachable = false
		if drone.Connectivity == "" || strings.EqualFold(drone.Connectivity, "Mesh") {
			drone.Connectivity = "Stale"
		}
		if !containsString(drone.HealthFlags, "stale_telemetry") {
			drone.HealthFlags = append(drone.HealthFlags, "stale_telemetry")
		}
	}
}

func containsString(values []string, target string) bool {
	for _, value := range values {
		if strings.EqualFold(strings.TrimSpace(value), target) {
			return true
		}
	}
	return false
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
	if t.LocalizationDataSource == "" {
		t.LocalizationDataSource = "unavailable"
	}
	t.Source = normalizeDroneSource(t.Source)
	t.Camera = sanitizeCameraTelemetry(t.Camera)
	t.IMU = sanitizeIMUTelemetry(t.IMU)
	t.LiDAR = sanitizeLiDARTelemetry(t.LiDAR)
	t.TDOA = sanitizeTDOATelemetry(t.TDOA)
	t.Replay = sanitizeReplayTelemetry(t.Replay)
	if t.Camera.Source == "unavailable" && (t.Camera.Status != "unavailable" || t.Camera.PreviewURL != "" || t.Camera.LatestFrameRef != "") {
		t.Camera.Source = t.Source
	}
	if t.IMU.Source == "unavailable" && (t.IMU.Status != "unavailable" || t.IMU.SampleRateHz > 0) {
		t.IMU.Source = t.Source
	}
	if t.LiDAR.Source == "unavailable" && (t.LiDAR.Status != "unavailable" || len(t.LiDAR.Points2D) > 0 || t.LiDAR.PointCount > 0) {
		t.LiDAR.Source = t.Source
	}
	if t.TDOA.Source == "unavailable" && (t.TDOA.Status != "unavailable" || len(t.TDOA.Anchors) > 0 || t.TDOA.VisibleAnchorCount > 0) {
		t.TDOA.Source = t.Source
	}
	if t.Replay.Source == "unavailable" && (t.Replay.Status != "unavailable" || t.Replay.Active || len(t.Replay.ConfidenceSeries) > 0) {
		t.Replay.Source = t.Source
	}
	if math.IsNaN(t.LocalizationConfidence) || math.IsInf(t.LocalizationConfidence, 0) {
		t.LocalizationConfidence = 0.0
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
	now := time.Now().UTC()
	for _, existing := range s.drones {
		drone := existing
		markDroneStale(&drone, s.staleAfter, now)
		drones = append(drones, drone)
	}
	sort.Slice(drones, func(i, j int) bool { return drones[i].DroneID < drones[j].DroneID })

	clusterAgg := map[string]*ClusterState{}
	var leaderID int
	var cpuSum float64
	var gpuSum float64
	var total int
	var critical int
	var realDroneCount int
	var staleDroneCount int
	for _, drone := range drones {
		total++
		cpuSum += drone.CPUTempC
		gpuSum += drone.GPULoadPct
		if drone.Source == "real" {
			realDroneCount++
		}
		if drone.Stale {
			staleDroneCount++
		}
		if drone.Role == "LEADER" && leaderID == 0 {
			leaderID = drone.DroneID
		}
		if drone.BatteryPct < 15 || drone.CPUTempC > 82 || !drone.Reachable || drone.Stale || drone.LocalizationState == "lost" || drone.SyncConfidence < 0.35 || (drone.SecurityState != "" && drone.SecurityState != "TRUSTED" && drone.SecurityState != "DEGRADED_LINK") {
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
		Drones:            drones,
		LeaderID:          leaderID,
		AvgLatencyMS:      4.0,
		PacketLossPct:     0.5,
		CPUTempC:          cpuAvg,
		GPULoadPct:        gpuAvg,
		Timestamp:         now,
		Clusters:          clusters,
		CriticalAlerts:    critical,
		BackendMode:       s.backendMode,
		SimulationEnabled: s.simulated,
		RealDroneCount:    realDroneCount,
		StaleDroneCount:   staleDroneCount,
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
		OnlineDrones:      online,
		TotalDrones:       len(snapshot.Drones),
		CriticalAlerts:    snapshot.CriticalAlerts,
		AvgBatteryPct:     avgBattery,
		MaxCPUTempC:       maxTemp,
		UpdatedAt:         time.Now().UTC(),
		BackendMode:       snapshot.BackendMode,
		SimulationEnabled: snapshot.SimulationEnabled,
		RealDroneCount:    snapshot.RealDroneCount,
		StaleDroneCount:   snapshot.StaleDroneCount,
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
	prevHash := s.lastAuditHash
	cmd.AuditPrevHash = prevHash
	cmd.AuditHash = auditHash("command", cmd.CommandID, cmd.Action, cmd.IssuedBy, cmd.Payload, prevHash, cmd.IssuedAt)
	s.lastAuditHash = cmd.AuditHash
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
	prevHash := s.lastAuditHash
	event.AuditPrevHash = prevHash
	event.AuditHash = auditHash("event", event.Type, event.Message, "", event.Data, prevHash, event.Timestamp)
	s.lastAuditHash = event.AuditHash
	s.events = append(s.events, event)
	if len(s.events) > 512 {
		s.events = append([]EventRecord(nil), s.events[len(s.events)-512:]...)
	}
}

func (s *FleetState) SavePendingApproval(approval PendingApproval) {
	log.Printf("FleetState.SavePendingApproval id=%s action=%s", approval.ApprovalID, approval.Action)
	s.mu.Lock()
	defer s.mu.Unlock()
	now := time.Now().UTC()
	for id, item := range s.approvals {
		if !item.ExpiresAt.IsZero() && item.ExpiresAt.Before(now) {
			delete(s.approvals, id)
		}
	}
	s.approvals[approval.ApprovalID] = approval
}

func (s *FleetState) PendingApproval(approvalID string) (PendingApproval, bool) {
	log.Printf("FleetState.PendingApproval id=%s", approvalID)
	s.mu.Lock()
	defer s.mu.Unlock()
	now := time.Now().UTC()
	for id, item := range s.approvals {
		if !item.ExpiresAt.IsZero() && item.ExpiresAt.Before(now) {
			delete(s.approvals, id)
		}
	}
	item, ok := s.approvals[approvalID]
	return item, ok
}

func (s *FleetState) DeletePendingApproval(approvalID string) {
	log.Printf("FleetState.DeletePendingApproval id=%s", approvalID)
	s.mu.Lock()
	defer s.mu.Unlock()
	delete(s.approvals, approvalID)
}

func (s *FleetState) PendingApprovals() []PendingApproval {
	log.Printf("FleetState.PendingApprovals requested")
	s.mu.Lock()
	defer s.mu.Unlock()
	now := time.Now().UTC()
	out := make([]PendingApproval, 0, len(s.approvals))
	for id, item := range s.approvals {
		if !item.ExpiresAt.IsZero() && item.ExpiresAt.Before(now) {
			delete(s.approvals, id)
			continue
		}
		out = append(out, item)
	}
	sort.Slice(out, func(i, j int) bool { return out[i].CreatedAt.Before(out[j].CreatedAt) })
	return out
}

func (s *FleetState) Events() []EventRecord {
	log.Printf("FleetState.Events requested")
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := make([]EventRecord, len(s.events))
	copy(out, s.events)
	return out
}

func auditHash(kind, subjectID, message, actor string, payload map[string]any, prevHash string, ts time.Time) string {
	if ts.IsZero() {
		ts = time.Now().UTC()
	}
	encoded, err := json.Marshal(payload)
	if err != nil {
		encoded = []byte("{}")
	}
	sum := sha256.Sum256([]byte(strings.Join([]string{
		kind,
		strings.TrimSpace(subjectID),
		strings.TrimSpace(message),
		strings.TrimSpace(actor),
		string(encoded),
		strings.TrimSpace(prevHash),
		ts.UTC().Format(time.RFC3339Nano),
	}, "\n")))
	return hex.EncodeToString(sum[:])
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
		DroneID:                 droneID,
		ClusterID:               clusterID,
		Role:                    role,
		Source:                  "simulation",
		Connectivity:            "Mesh",
		Reachable:               true,
		Position:                [3]float64{float64(droneID % 20), float64((droneID / 20) % 10), 8.0 + float64(droneID%3)},
		Velocity:                [3]float64{0.0, 0.0, 0.0},
		AttitudeRPY:             [3]float64{0.0, 0.0, 0.0},
		ThrustVector:            [3]float64{0.0, 0.0, 9.81},
		CommandedAltitudeM:      8.0,
		CommandedSpeedMPS:       3.0,
		ManualTargetPosition:    [3]float64{float64(droneID % 20), float64((droneID / 20) % 10), 8.0 + float64(droneID%3)},
		ManualControlActive:     false,
		DriftM:                  0.05,
		BatteryPct:              96.0,
		RSSIDBm:                 -50.0,
		CPUTempC:                54.0,
		GPULoadPct:              38.0,
		MissionState:            "standby",
		LocalizationSource:      "vision-inertial",
		LocalizationDataSource:  "simulation",
		LocalizationState:       "nominal",
		LocalizationConfidence:  0.92,
		TDOAConfidence:          0.66,
		ConfidenceTrend:         0.03,
		RelocalizationCount:     0,
		VisibleAnchorCount:      5,
		OccupancyRatio:          0.14,
		SyncConfidence:          0.93,
		IMUCameraOffsetMS:       2.1,
		SecurityState:           "TRUSTED",
		SecuritySummary:         "All trust signals nominal",
		RemoteCommandAllowed:    true,
		TelemetryUplinkAllowed:  true,
		LinkIntegrityScore:      0.90,
		FirmwareMeasurement:     "lab-local-build",
		FirmwareVersion:         "2.0.0",
		SecureBootState:         "LAB_BOOT",
		BootTrustSummary:        "Lab digital twin boot trust state",
		RollbackCounter:         1,
		MaintenanceMode:         false,
		UpdateChannelState:      "idle",
		LastRemoteCommandStatus: "no remote command",
		Camera: CameraTelemetry{
			Status:         "simulation",
			FPS:            15.0,
			FrameAgeMS:     50.0,
			Resolution:     "1280x720",
			DroppedFrames:  0,
			Source:         "simulation",
			LatestFrameRef: fmt.Sprintf("sim-add-%d", droneID),
		},
		IMU: IMUTelemetry{
			Status:          "simulation",
			SampleRateHz:    100.0,
			LastSampleAgeMS: 12.0,
			Accel:           Vector3{X: 0.0, Y: 0.0, Z: 9.81},
			Gyro:            Vector3{X: 0.0, Y: 0.0, Z: 0.0},
			Health:          "simulation",
			Source:          "simulation",
		},
		LiDAR: LiDARTelemetry{
			Status:       "simulation",
			PacketRateHz: 8.0,
			ScanAgeMS:    60.0,
			PointCount:   0,
			MinRangeM:    0.3,
			MaxRangeM:    20.0,
			Source:       "simulation",
		},
		TDOA: TDOATelemetry{
			Status:             "simulation",
			Source:             "simulation",
			VisibleAnchorCount: 4,
			CalibrationWarning: "simulation anchors active",
		},
		Replay: ReplayTelemetry{
			Status: "simulation",
			Source: "simulation",
		},
		Timestamp: time.Now().UTC(),
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
			drone.ManualControlActive = false
			if shape, ok := payload["shape"].(string); ok {
				drone.MissionState = "formation-" + strings.ToLower(shape)
				s.clusterForm[drone.ClusterID] = shape
			} else {
				drone.MissionState = "formation-hold"
			}
		case "hold_position":
			drone.ManualControlActive = true
			drone.ManualTargetPosition = drone.Position
			drone.MissionState = "hold-position"
		case "move_left":
			step := movementStep(payload)
			base := drone.Position
			if drone.ManualControlActive {
				base = drone.ManualTargetPosition
			}
			base[0] -= step
			drone.ManualTargetPosition = base
			drone.ManualControlActive = true
			drone.MissionState = "remote-nav"
		case "move_right":
			step := movementStep(payload)
			base := drone.Position
			if drone.ManualControlActive {
				base = drone.ManualTargetPosition
			}
			base[0] += step
			drone.ManualTargetPosition = base
			drone.ManualControlActive = true
			drone.MissionState = "remote-nav"
		case "move_up":
			step := movementStep(payload)
			base := drone.Position
			if drone.ManualControlActive {
				base = drone.ManualTargetPosition
			}
			base[1] += step
			drone.ManualTargetPosition = base
			drone.ManualControlActive = true
			drone.MissionState = "remote-nav"
		case "move_down":
			step := movementStep(payload)
			base := drone.Position
			if drone.ManualControlActive {
				base = drone.ManualTargetPosition
			}
			base[1] -= step
			drone.ManualTargetPosition = base
			drone.ManualControlActive = true
			drone.MissionState = "remote-nav"
		case "return_home":
			drone.ManualControlActive = false
			drone.MissionState = "return-home"
		case "emergency_land":
			drone.ManualControlActive = false
			drone.MissionState = "emergency-land"
		case "fly":
			drone.ManualControlActive = false
			drone.MissionState = "fly"
		case "land":
			drone.ManualControlActive = false
			drone.MissionState = "land"
		case "election":
			drone.ManualControlActive = false
			drone.MissionState = "leader-election"
		case "maintenance_mode":
			drone.ManualControlActive = false
			drone.MaintenanceMode = true
			drone.UpdateChannelState = "maintenance-window-open"
			drone.MissionState = "maintenance-window"
		case "firmware_update":
			drone.ManualControlActive = false
			drone.MaintenanceMode = true
			drone.MissionState = "firmware-update"
			if value, ok := payload["firmware_version"].(string); ok && strings.TrimSpace(value) != "" {
				drone.FirmwareVersion = strings.TrimSpace(value)
			}
			if value, ok := payload["firmware_measurement"].(string); ok && strings.TrimSpace(value) != "" {
				drone.FirmwareMeasurement = strings.TrimSpace(value)
			}
			if value, ok := payload["secure_boot_state"].(string); ok && strings.TrimSpace(value) != "" {
				drone.SecureBootState = strings.TrimSpace(value)
			} else {
				drone.SecureBootState = "SECURE_BOOT_TRUSTED"
			}
			if value, ok := payload["boot_trust_summary"].(string); ok && strings.TrimSpace(value) != "" {
				drone.BootTrustSummary = strings.TrimSpace(value)
			} else {
				drone.BootTrustSummary = "Firmware update applied through trusted maintenance workflow"
			}
			if value, ok := asFloat64(payload["rollback_counter"]); ok && value >= 0 {
				drone.RollbackCounter = uint64(value)
			} else {
				drone.RollbackCounter++
			}
			drone.UpdateChannelState = "firmware-updated"
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
