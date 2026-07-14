package controlplane

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"
)

func newTestServer() *Server {
	return NewServer(":0", SecurityConfig{Profile: "lab"}, TLSConfig{}, ServerConfig{
		Mode:              "production",
		SimulationEnabled: false,
		StaleAfter:        50 * time.Millisecond,
	})
}

func newSimulationTestServer() *Server {
	return NewServer(":0", SecurityConfig{Profile: "lab"}, TLSConfig{}, ServerConfig{
		Mode:              "simulation",
		SimulationEnabled: true,
		StaleAfter:        50 * time.Millisecond,
		DemoFleetSize:     5,
	})
}

func TestTelemetryIngestAndFleetSnapshotRoundTrip(t *testing.T) {
	server := newTestServer()

	body, err := json.Marshal(DroneTelemetry{
		DroneID:                42,
		ClusterID:              "cluster-03",
		Role:                   "LEADER",
		Connectivity:           "Mesh",
		Reachable:              true,
		Position:               [3]float64{1.0, 2.0, 3.0},
		Velocity:               [3]float64{0.1, 0.0, 0.0},
		AttitudeRPY:            [3]float64{0.0, 0.0, 0.2},
		ThrustVector:           [3]float64{0.0, 0.0, 9.81},
		MissionState:           "fly",
		Source:                 "playback",
		LocalizationSource:     "vision-depth-fused",
		LocalizationState:      "nominal",
		LocalizationConfidence: 0.91,
		TDOAConfidence:         0.73,
		SyncConfidence:         0.95,
		Camera: CameraTelemetry{
			Status:         "live",
			FPS:            29.8,
			FrameAgeMS:     34.0,
			Resolution:     "1280x720",
			DroppedFrames:  2,
			Source:         "playback",
			PreviewURL:     "http://127.0.0.1:9090/preview/42.jpg",
			LatestFrameRef: "frame-42",
		},
		IMU: IMUTelemetry{
			Status:          "live",
			SampleRateHz:    200.0,
			LastSampleAgeMS: 4.0,
			Accel:           Vector3{X: 0.01, Y: -0.02, Z: 9.82},
			Gyro:            Vector3{X: 0.1, Y: 0.0, Z: -0.1},
			Health:          "good",
			Source:          "playback",
		},
		LiDAR: LiDARTelemetry{
			Status:       "playback",
			PacketRateHz: 10.0,
			ScanAgeMS:    55.0,
			PointCount:   2,
			Points2D: []LidarPoint2D{
				{X: 1.0, Y: 2.0, Intensity: 0.8},
				{X: -1.0, Y: 0.5, Intensity: 0.4},
			},
			MinRangeM: 0.3,
			MaxRangeM: 20.0,
			Source:    "playback",
		},
		TDOA: TDOATelemetry{
			Status:             "playback",
			Source:             "playback",
			VisibleAnchorCount: 2,
			Anchors: []TDOAAnchorTelemetry{
				{ID: "A0", X: 0, Y: 0, Z: 2.5, Visible: true, LastSeenMS: 20},
				{ID: "A1", X: 8, Y: 0, Z: 2.5, Visible: true, LastSeenMS: 22},
			},
			EstimatedPosition:  Vector3{X: 1.0, Y: 2.0, Z: 3.0},
			CalibrationWarning: "playback anchor layout",
		},
		Replay: ReplayTelemetry{
			Status:           "playback",
			Active:           true,
			FileName:         "session-42.log",
			Progress:         0.4,
			CurrentTime:      12.5,
			ConfidenceSeries: []float64{0.9, 0.88, 0.86},
			Source:           "playback",
		},
	})
	if err != nil {
		t.Fatalf("marshal telemetry: %v", err)
	}

	req := httptest.NewRequest(http.MethodPost, "/api/v1/telemetry", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	rec := httptest.NewRecorder()
	server.http.Handler.ServeHTTP(rec, req)
	if rec.Code != http.StatusAccepted {
		t.Fatalf("expected telemetry accepted, got %d body=%s", rec.Code, rec.Body.String())
	}

	fleetReq := httptest.NewRequest(http.MethodGet, "/api/v1/fleet?cluster_id=cluster-03", nil)
	fleetRec := httptest.NewRecorder()
	server.http.Handler.ServeHTTP(fleetRec, fleetReq)
	if fleetRec.Code != http.StatusOK {
		t.Fatalf("expected fleet snapshot ok, got %d", fleetRec.Code)
	}

	var snapshot FleetSnapshot
	if err := json.Unmarshal(fleetRec.Body.Bytes(), &snapshot); err != nil {
		t.Fatalf("unmarshal fleet snapshot: %v", err)
	}
	found := false
	for _, drone := range snapshot.Drones {
		if drone.DroneID == 42 {
			found = true
			if drone.LocalizationSource != "vision-depth-fused" {
				t.Fatalf("expected localization source to round-trip, got %q", drone.LocalizationSource)
			}
			if drone.Source != "playback" {
				t.Fatalf("expected telemetry source to round-trip, got %q", drone.Source)
			}
			if drone.Camera.PreviewURL == "" || drone.Camera.Source != "playback" {
				t.Fatalf("expected camera payload to round-trip, got %+v", drone.Camera)
			}
			if len(drone.LiDAR.Points2D) != 2 {
				t.Fatalf("expected lidar points to round-trip, got %+v", drone.LiDAR)
			}
			if len(drone.TDOA.Anchors) != 2 || drone.TDOA.Source != "playback" {
				t.Fatalf("expected tdoa payload to round-trip, got %+v", drone.TDOA)
			}
			if !drone.Replay.Active || drone.Replay.Source != "playback" {
				t.Fatalf("expected replay payload to round-trip, got %+v", drone.Replay)
			}
		}
	}
	if !found {
		t.Fatal("expected ingested drone to appear in fleet snapshot")
	}
	if snapshot.BackendMode != "production" {
		t.Fatalf("expected production backend mode, got %q", snapshot.BackendMode)
	}
	if snapshot.SimulationEnabled {
		t.Fatal("expected production backend to disable simulation")
	}
	if snapshot.RealDroneCount != 0 {
		t.Fatalf("expected no real drones when only playback telemetry is ingested, got %d", snapshot.RealDroneCount)
	}
}

func TestTelemetryIngestTruncatesOversizedSensorArrays(t *testing.T) {
	server := newTestServer()
	points := make([]LidarPoint2D, 0, maxLidarPoints2D+40)
	confidence := make([]float64, 0, maxReplayConfidenceCount+20)
	for idx := 0; idx < maxLidarPoints2D+40; idx++ {
		points = append(points, LidarPoint2D{X: float64(idx), Y: float64(idx) * 0.5, Intensity: 0.5})
	}
	for idx := 0; idx < maxReplayConfidenceCount+20; idx++ {
		confidence = append(confidence, 0.75)
	}

	body, err := json.Marshal(DroneTelemetry{
		DroneID:      88,
		ClusterID:    "cluster-09",
		Role:         "FOLLOWER",
		Source:       "real",
		Connectivity: "Mesh",
		Reachable:    true,
		Position:     [3]float64{0, 0, 1},
		Velocity:     [3]float64{0, 0, 0},
		AttitudeRPY:  [3]float64{},
		ThrustVector: [3]float64{0, 0, 9.81},
		MissionState: "hold-position",
		LiDAR:        LiDARTelemetry{Status: "live", Source: "real", Points2D: points, PointCount: len(points)},
		Replay:       ReplayTelemetry{Status: "playback", Active: true, Source: "playback", ConfidenceSeries: confidence},
		Timestamp:    time.Now().UTC(),
	})
	if err != nil {
		t.Fatalf("marshal telemetry: %v", err)
	}

	req := httptest.NewRequest(http.MethodPost, "/api/v1/telemetry", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	rec := httptest.NewRecorder()
	server.http.Handler.ServeHTTP(rec, req)
	if rec.Code != http.StatusAccepted {
		t.Fatalf("expected telemetry accepted, got %d body=%s", rec.Code, rec.Body.String())
	}

	snapshot := server.state.Snapshot()
	if len(snapshot.Drones) != 1 {
		t.Fatalf("expected one drone, got %d", len(snapshot.Drones))
	}
	if got := len(snapshot.Drones[0].LiDAR.Points2D); got != maxLidarPoints2D {
		t.Fatalf("expected lidar points truncated to %d, got %d", maxLidarPoints2D, got)
	}
	if got := len(snapshot.Drones[0].Replay.ConfidenceSeries); got != maxReplayConfidenceCount {
		t.Fatalf("expected replay confidence truncated to %d, got %d", maxReplayConfidenceCount, got)
	}
}

func TestHealthEndpointReflectsCriticalLocalizationLoss(t *testing.T) {
	server := newTestServer()
	server.state.UpsertTelemetry(DroneTelemetry{
		DroneID:           77,
		ClusterID:         "cluster-04",
		Role:              "FOLLOWER",
		Connectivity:      "Degraded",
		Reachable:         false,
		Position:          [3]float64{0, 0, 0},
		Velocity:          [3]float64{0, 0, 0},
		AttitudeRPY:       [3]float64{},
		ThrustVector:      [3]float64{0, 0, 9.81},
		MissionState:      "hold-position",
		BatteryPct:        12.0,
		CPUTempC:          83.0,
		LocalizationState: "lost",
		SyncConfidence:    0.2,
		SecurityState:     "SAFE_RETURN",
	})

	req := httptest.NewRequest(http.MethodGet, "/api/v1/health", nil)
	rec := httptest.NewRecorder()
	server.http.Handler.ServeHTTP(rec, req)
	if rec.Code != http.StatusOK {
		t.Fatalf("expected health ok, got %d", rec.Code)
	}

	var health HealthReport
	if err := json.Unmarshal(rec.Body.Bytes(), &health); err != nil {
		t.Fatalf("unmarshal health: %v", err)
	}
	if health.CriticalAlerts == 0 {
		t.Fatal("expected critical alerts when lost localization telemetry is present")
	}
}

func TestProductionModeDoesNotSeedFakeDrones(t *testing.T) {
	server := newTestServer()

	req := httptest.NewRequest(http.MethodGet, "/api/v1/fleet", nil)
	rec := httptest.NewRecorder()
	server.http.Handler.ServeHTTP(rec, req)
	if rec.Code != http.StatusOK {
		t.Fatalf("expected fleet snapshot ok, got %d", rec.Code)
	}

	var snapshot FleetSnapshot
	if err := json.Unmarshal(rec.Body.Bytes(), &snapshot); err != nil {
		t.Fatalf("unmarshal fleet snapshot: %v", err)
	}
	if len(snapshot.Drones) != 0 {
		t.Fatalf("expected no seeded drones in production mode, got %d", len(snapshot.Drones))
	}
	if snapshot.BackendMode != "production" {
		t.Fatalf("expected production backend mode, got %q", snapshot.BackendMode)
	}
	if snapshot.SimulationEnabled {
		t.Fatal("expected simulation to be disabled in production")
	}
}

func TestSimulationModeSeedsDemoFleet(t *testing.T) {
	server := newSimulationTestServer()
	snapshot := server.state.Snapshot()
	if len(snapshot.Drones) == 0 {
		t.Fatal("expected simulation mode to seed demo fleet")
	}
	if snapshot.BackendMode != "simulation" {
		t.Fatalf("expected simulation backend mode, got %q", snapshot.BackendMode)
	}
	if !snapshot.SimulationEnabled {
		t.Fatal("expected simulation backend to enable simulation loop")
	}
	foundSimulation := false
	for _, drone := range snapshot.Drones {
		if drone.Source == "simulation" {
			foundSimulation = true
			break
		}
	}
	if !foundSimulation {
		t.Fatal("expected demo fleet drones to be marked as simulation")
	}
}

func TestStaleTelemetryIsMarkedStale(t *testing.T) {
	server := newTestServer()
	server.state.UpsertTelemetry(DroneTelemetry{
		DroneID:                91,
		ClusterID:              "cluster-05",
		Role:                   "FOLLOWER",
		Source:                 "real",
		Connectivity:           "Mesh",
		Reachable:              true,
		Position:               [3]float64{0.0, 0.0, 0.0},
		Velocity:               [3]float64{0.0, 0.0, 0.0},
		AttitudeRPY:            [3]float64{0.0, 0.0, 0.0},
		ThrustVector:           [3]float64{0.0, 0.0, 9.81},
		MissionState:           "hold-position",
		LocalizationSource:     "vision-inertial",
		LocalizationDataSource: "real",
		LocalizationState:      "nominal",
		LocalizationConfidence: 0.84,
		SyncConfidence:         0.91,
		Timestamp:              time.Now().UTC().Add(-500 * time.Millisecond),
	})

	snapshot := server.state.Snapshot()
	if snapshot.StaleDroneCount != 1 {
		t.Fatalf("expected one stale drone, got %d", snapshot.StaleDroneCount)
	}
	if snapshot.RealDroneCount != 1 {
		t.Fatalf("expected one real drone, got %d", snapshot.RealDroneCount)
	}
	if len(snapshot.Drones) != 1 {
		t.Fatalf("expected one drone in snapshot, got %d", len(snapshot.Drones))
	}
	if !snapshot.Drones[0].Stale {
		t.Fatal("expected drone to be marked stale")
	}
	if snapshot.Drones[0].Source != "real" {
		t.Fatalf("expected real drone source, got %q", snapshot.Drones[0].Source)
	}
}

func TestFleetResponseIncludesSourceFields(t *testing.T) {
	server := newTestServer()
	server.state.UpsertTelemetry(DroneTelemetry{
		DroneID:                17,
		ClusterID:              "cluster-02",
		Role:                   "LEADER",
		Source:                 "real",
		Connectivity:           "Mesh",
		Reachable:              true,
		Position:               [3]float64{1.0, 1.0, 1.0},
		Velocity:               [3]float64{0.1, 0.0, 0.0},
		AttitudeRPY:            [3]float64{0.0, 0.0, 0.0},
		ThrustVector:           [3]float64{0.0, 0.0, 9.81},
		MissionState:           "fly",
		LocalizationSource:     "vision-inertial",
		LocalizationDataSource: "real",
		LocalizationState:      "nominal",
		LocalizationConfidence: 0.93,
		SyncConfidence:         0.95,
		Timestamp:              time.Now().UTC(),
	})

	req := httptest.NewRequest(http.MethodGet, "/api/v1/fleet", nil)
	rec := httptest.NewRecorder()
	server.http.Handler.ServeHTTP(rec, req)
	if rec.Code != http.StatusOK {
		t.Fatalf("expected fleet snapshot ok, got %d", rec.Code)
	}

	var snapshot FleetSnapshot
	if err := json.Unmarshal(rec.Body.Bytes(), &snapshot); err != nil {
		t.Fatalf("unmarshal fleet snapshot: %v", err)
	}
	if snapshot.RealDroneCount != 1 {
		t.Fatalf("expected fleet response to include one real drone, got %d", snapshot.RealDroneCount)
	}
	if snapshot.StaleDroneCount != 0 {
		t.Fatalf("expected no stale drones, got %d", snapshot.StaleDroneCount)
	}
	if len(snapshot.Drones) != 1 || snapshot.Drones[0].Source != "real" {
		t.Fatalf("expected fleet drone source to be present, got %+v", snapshot.Drones)
	}
}

func TestCommandsEndpointRecordsEventAndStateChange(t *testing.T) {
	server := newSimulationTestServer()

	body := []byte(`{"action":"return_home","payload":{"target_ids":[1]}}`)
	req := httptest.NewRequest(http.MethodPost, "/api/v1/commands", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	rec := httptest.NewRecorder()
	server.http.Handler.ServeHTTP(rec, req)
	if rec.Code != http.StatusAccepted {
		t.Fatalf("expected command accepted, got %d body=%s", rec.Code, rec.Body.String())
	}

	snapshot := server.state.Snapshot()
	var missionState string
	for _, drone := range snapshot.Drones {
		if drone.DroneID == 1 {
			missionState = drone.MissionState
			break
		}
	}
	if missionState != "return-home" {
		t.Fatalf("expected target drone mission state to change, got %q", missionState)
	}

	eventsReq := httptest.NewRequest(http.MethodGet, "/api/v1/events?limit=10", nil)
	eventsRec := httptest.NewRecorder()
	server.http.Handler.ServeHTTP(eventsRec, eventsReq)
	if eventsRec.Code != http.StatusOK {
		t.Fatalf("expected events ok, got %d", eventsRec.Code)
	}

	var payload struct {
		Events []EventRecord `json:"events"`
	}
	if err := json.Unmarshal(eventsRec.Body.Bytes(), &payload); err != nil {
		t.Fatalf("unmarshal events: %v", err)
	}
	foundCommandEvent := false
	for _, event := range payload.Events {
		if event.Type == "command" {
			foundCommandEvent = true
			break
		}
	}
	if !foundCommandEvent {
		t.Fatal("expected command event after accepted command")
	}
}
