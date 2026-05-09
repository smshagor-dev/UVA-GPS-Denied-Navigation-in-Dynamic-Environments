package controlplane

import (
	"encoding/json"
	"math"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

func TestApplyCommandFlyAndLandUpdateMissionAndTunables(t *testing.T) {
	state := NewFleetState()
	state.UpsertTelemetry(DroneTelemetry{
		DroneID:            1,
		ClusterID:          "cluster-01",
		Role:               "LEADER",
		Connectivity:       "Mesh",
		Reachable:          true,
		Position:           [3]float64{0, 0, 0},
		Velocity:           [3]float64{0, 0, 0},
		AttitudeRPY:        [3]float64{},
		ThrustVector:       [3]float64{0, 0, 9.81},
		CommandedAltitudeM: 6.0,
		CommandedSpeedMPS:  2.0,
		MissionState:       "standby",
	})

	affected := state.ApplyCommand("fly", "", []int{1}, map[string]any{
		"altitude_m":   12.0,
		"velocity_mps": 4.5,
	})
	if affected != 1 {
		t.Fatalf("expected 1 affected drone, got %d", affected)
	}

	snapshot := state.Snapshot()
	drone := snapshot.Drones[0]
	if drone.MissionState != "fly" {
		t.Fatalf("expected fly mission state, got %q", drone.MissionState)
	}
	if drone.CommandedAltitudeM != 12.0 {
		t.Fatalf("expected altitude 12.0, got %.2f", drone.CommandedAltitudeM)
	}
	if drone.CommandedSpeedMPS != 4.5 {
		t.Fatalf("expected speed 4.5, got %.2f", drone.CommandedSpeedMPS)
	}

	state.ApplyCommand("land", "", []int{1}, map[string]any{})
	snapshot = state.Snapshot()
	if snapshot.Drones[0].MissionState != "land" {
		t.Fatalf("expected land mission state, got %q", snapshot.Drones[0].MissionState)
	}
}

func TestApplyCommandManualMoveUpdatesRemoteTarget(t *testing.T) {
	state := NewFleetState()
	state.UpsertTelemetry(DroneTelemetry{
		DroneID:              7,
		ClusterID:            "cluster-01",
		Role:                 "FOLLOWER",
		Connectivity:         "Mesh",
		Reachable:            true,
		Position:             [3]float64{2, 3, 5},
		Velocity:             [3]float64{0, 0, 0},
		AttitudeRPY:          [3]float64{},
		ThrustVector:         [3]float64{0, 0, 9.81},
		CommandedAltitudeM:   5.0,
		CommandedSpeedMPS:    2.0,
		ManualTargetPosition: [3]float64{2, 3, 5},
		ManualControlActive:  false,
		MissionState:         "formation-hold",
	})

	affected := state.ApplyCommand("move_right", "", []int{7}, map[string]any{
		"step_m":       2.5,
		"velocity_mps": 4.0,
	})
	if affected != 1 {
		t.Fatalf("expected 1 affected drone, got %d", affected)
	}

	snapshot := state.Snapshot()
	drone := snapshot.Drones[0]
	if !drone.ManualControlActive {
		t.Fatal("expected manual control to be active")
	}
	if drone.MissionState != "remote-nav" {
		t.Fatalf("expected remote-nav mission state, got %q", drone.MissionState)
	}
	if drone.ManualTargetPosition[0] <= drone.Position[0] {
		t.Fatalf("expected target x to move right, pos=%+v target=%+v", drone.Position, drone.ManualTargetPosition)
	}
}

func TestLeaderFigureEightIsDynamic(t *testing.T) {
	p0, v0 := leaderFigureEight(0)
	p1, v1 := leaderFigureEight(4.0)
	if p0 == p1 {
		t.Fatal("leader path should change over time")
	}
	if vecNorm(v0) == 0 || vecNorm(v1) == 0 {
		t.Fatal("leader velocity should stay dynamic")
	}
}

func TestGuidanceCommandProducesTiltAndForwardAcceleration(t *testing.T) {
	position := [3]float64{0, 0, 0}
	velocity := [3]float64{0, 0, 0}
	target := [3]float64{10, 0, 5}
	targetVel := [3]float64{1, 0, 0}

	accel, thrust, attitude := computeGuidanceCommand(position, velocity, target, targetVel, 4.0, 3.0)
	if accel[0] <= 0 {
		t.Fatalf("expected forward acceleration, got %+v", accel)
	}
	if thrust[2] <= 9.81 {
		t.Fatalf("expected upward thrust compensation, got %+v", thrust)
	}
	if math.Abs(attitude[1]) < 1e-6 {
		t.Fatalf("expected non-zero pitch, got %+v", attitude)
	}
}

func TestIntegrateDroneKinematicsMovesTowardTarget(t *testing.T) {
	position := [3]float64{0, 0, 0}
	velocity := [3]float64{0, 0, 0}
	target := [3]float64{0, 0, 8}

	nextPos, nextVel, thrust, attitude := integrateDroneKinematics(position, velocity, target, [3]float64{}, 0.1, 3.0, 2.0)
	if nextPos[2] <= position[2] {
		t.Fatalf("expected altitude increase, got pos %+v", nextPos)
	}
	if nextVel[2] <= 0 {
		t.Fatalf("expected upward velocity, got %+v", nextVel)
	}
	if thrust[2] <= 9.81 {
		t.Fatalf("expected upward thrust, got %+v", thrust)
	}
	if math.IsNaN(attitude[0]) || math.IsNaN(attitude[1]) || math.IsNaN(attitude[2]) {
		t.Fatalf("unexpected NaN attitude %+v", attitude)
	}
}

func TestCommandRequiresApprovalForCriticalActions(t *testing.T) {
	if !CommandRequiresApproval("election") {
		t.Fatal("expected election to require approval")
	}
	if !CommandRequiresApproval("emergency_land") {
		t.Fatal("expected emergency_land to require approval")
	}
	if !CommandRequiresApproval("firmware_update") {
		t.Fatal("expected firmware_update to require approval")
	}
	if CommandRequiresApproval("fly") {
		t.Fatal("did not expect fly to require approval")
	}
}

func TestValidateMaintenanceWorkflowRejectsIncompleteFirmwareUpdate(t *testing.T) {
	err := validateMaintenanceWorkflow(ValidatedCommand{
		Action: "firmware_update",
		Payload: map[string]any{
			"firmware_version": "2.1.0",
		},
	})
	if err == nil {
		t.Fatal("expected firmware_update validation to fail when maintenance fields are missing")
	}
}

func TestValidateMaintenanceWorkflowAcceptsTrustedFirmwareUpdate(t *testing.T) {
	err := validateMaintenanceWorkflow(ValidatedCommand{
		Action: "firmware_update",
		Payload: map[string]any{
			"maintenance_token_id": "mw-001",
			"maintenance_window":   true,
			"firmware_version":     "2.1.0",
			"firmware_measurement": "fw-secure-2026-04-17",
			"firmware_signer":      "release-ca",
			"firmware_signature":   "abcdef1234",
			"rollback_counter":     7.0,
		},
	})
	if err != nil {
		t.Fatalf("expected trusted firmware update payload to validate, got %v", err)
	}
}

func TestPayloadWithoutApprovalRemovesApprovalID(t *testing.T) {
	payload := map[string]any{
		"approval_id": "approval-123",
		"cluster_id":  "cluster-01",
		"target_ids":  []int{1, 2},
	}
	sanitized := payloadWithoutApproval(payload)
	if _, ok := sanitized["approval_id"]; ok {
		t.Fatal("approval_id should be stripped")
	}
	encoded, err := json.Marshal(sanitized)
	if err != nil {
		t.Fatalf("unexpected marshal error: %v", err)
	}
	if string(encoded) == "" {
		t.Fatal("expected sanitized payload to remain serializable")
	}
}

func TestTelemetrySecurityFieldsFlowIntoFleetSnapshot(t *testing.T) {
	server := NewServer(":0", SecurityConfig{Profile: "lab"}, TLSConfig{})

	body := `{
		"drone_id": 21,
		"cluster_id": "cluster-02",
		"role": "FOLLOWER",
		"connectivity": "Mesh",
		"reachable": true,
		"position": [1, 2, 3],
		"velocity": [0.1, 0.2, 0.3],
		"attitude_rpy": [0, 0, 0],
		"thrust_vector": [0, 0, 9.81],
		"drift_m": 0.42,
		"battery_pct": 61,
		"rssi_dbm": -58,
		"cpu_temp_c": 54,
		"gpu_load_pct": 23,
		"mission_state": "hold",
		"security_state": "SAFE_RETURN",
		"security_summary": "Navigation trust degraded with unstable link, returning via safe autonomy",
		"security_transition_reason": "safety-consistency-failed",
		"remote_command_allowed": false,
		"telemetry_uplink_allowed": true,
		"link_integrity_score": 0.29,
		"trust_epoch": 4,
		"last_auth_failure_at_s": 18.5,
		"tamper_score": 0.41,
		"firmware_measurement": "fw-secure-2026-04-17",
		"last_remote_command_status": "RETURN_HOME rejected (Remote command rejected by onboard security state)",
		"health_flags": ["security-safe-return", "remote-command-blocked"]
	}`

	telemetryReq := httptest.NewRequest(http.MethodPost, "/api/v1/telemetry", strings.NewReader(body))
	telemetryRec := httptest.NewRecorder()
	server.handleTelemetry(telemetryRec, telemetryReq)
	if telemetryRec.Code != http.StatusAccepted {
		t.Fatalf("expected accepted telemetry, got %d", telemetryRec.Code)
	}

	fleetReq := httptest.NewRequest(http.MethodGet, "/api/v1/fleet", nil)
	fleetRec := httptest.NewRecorder()
	server.handleFleet(fleetRec, fleetReq)
	if fleetRec.Code != http.StatusOK {
		t.Fatalf("expected fleet snapshot response, got %d", fleetRec.Code)
	}

	var snapshot FleetSnapshot
	if err := json.NewDecoder(fleetRec.Body).Decode(&snapshot); err != nil {
		t.Fatalf("decode fleet snapshot: %v", err)
	}

	var found *DroneTelemetry
	for i := range snapshot.Drones {
		if snapshot.Drones[i].DroneID == 21 {
			found = &snapshot.Drones[i]
			break
		}
	}
	if found == nil {
		t.Fatal("expected ingested drone telemetry to appear in fleet snapshot")
	}
	if found.SecurityState != "SAFE_RETURN" {
		t.Fatalf("expected security state SAFE_RETURN, got %q", found.SecurityState)
	}
	if found.RemoteCommandAllowed {
		t.Fatal("expected remote commands to remain blocked in fleet snapshot")
	}
	if found.LinkIntegrityScore != 0.29 {
		t.Fatalf("expected link integrity 0.29, got %.2f", found.LinkIntegrityScore)
	}
	if found.FirmwareMeasurement != "fw-secure-2026-04-17" {
		t.Fatalf("expected firmware measurement to round-trip, got %q", found.FirmwareMeasurement)
	}
	if len(found.HealthFlags) != 2 || found.HealthFlags[0] != "security-safe-return" {
		t.Fatalf("expected health flags to round-trip, got %+v", found.HealthFlags)
	}
	if snapshot.CriticalAlerts == 0 {
		t.Fatal("expected non-trusted security state to raise critical alert count")
	}
}

func TestAuditHashesChainAcrossEventsAndCommands(t *testing.T) {
	state := NewFleetState()
	firstEvent := EventRecord{
		Type:      "security",
		Message:   "initial event",
		Timestamp: time.Now().UTC(),
		Data:      map[string]any{"step": 1},
	}
	state.AddEvent(firstEvent)
	events := state.Events()
	if len(events) != 1 {
		t.Fatalf("expected 1 event, got %d", len(events))
	}
	if events[0].AuditHash == "" {
		t.Fatal("expected first event to have an audit hash")
	}
	if events[0].AuditPrevHash != "" {
		t.Fatal("expected first event to start a new audit chain")
	}

	cmd := CommandEnvelope{
		CommandID: "cmd-1",
		Action:    "fly",
		Payload:   map[string]any{"cluster_id": "cluster-01"},
		IssuedBy:  "operator:operator-console-1",
		IssuedAt:  time.Now().UTC(),
	}
	state.RecordCommand(cmd)
	commands := state.Commands()
	if len(commands) != 1 {
		t.Fatalf("expected 1 command, got %d", len(commands))
	}
	if commands[0].AuditHash == "" {
		t.Fatal("expected command to have an audit hash")
	}
	if commands[0].AuditPrevHash != events[0].AuditHash {
		t.Fatalf("expected command to chain from prior event, got prev=%q event=%q", commands[0].AuditPrevHash, events[0].AuditHash)
	}

	state.AddEvent(EventRecord{
		Type:      "command",
		Message:   "follow-up event",
		Timestamp: time.Now().UTC(),
		Data:      map[string]any{"command_id": "cmd-1"},
	})
	events = state.Events()
	last := events[len(events)-1]
	if last.AuditPrevHash != commands[0].AuditHash {
		t.Fatalf("expected follow-up event to chain from command hash, got prev=%q command=%q", last.AuditPrevHash, commands[0].AuditHash)
	}
}
