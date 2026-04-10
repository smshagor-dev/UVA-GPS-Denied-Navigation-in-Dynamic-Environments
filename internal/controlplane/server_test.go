package controlplane

import (
	"math"
	"encoding/json"
	"testing"
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
	if CommandRequiresApproval("fly") {
		t.Fatal("did not expect fly to require approval")
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
