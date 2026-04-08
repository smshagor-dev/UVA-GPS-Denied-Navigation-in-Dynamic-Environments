// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

package controlplane

import "time"

type DroneTelemetry struct {
	DroneID       int       `json:"drone_id"`
	ClusterID     string    `json:"cluster_id"`
	Role          string    `json:"role"`
	Connectivity  string    `json:"connectivity"`
	Reachable     bool      `json:"reachable"`
	Position      [3]float64 `json:"position"`
	Velocity      [3]float64 `json:"velocity"`
	DriftM        float64   `json:"drift_m"`
	BatteryPct    float64   `json:"battery_pct"`
	RSSIDBm       float64   `json:"rssi_dbm"`
	CPUTempC      float64   `json:"cpu_temp_c"`
	GPULoadPct    float64   `json:"gpu_load_pct"`
	MissionState  string    `json:"mission_state"`
	HealthFlags   []string  `json:"health_flags,omitempty"`
	Timestamp     time.Time `json:"timestamp"`
}

type FleetSnapshot struct {
	Drones         []DroneTelemetry `json:"drones"`
	LeaderID       int              `json:"leader_id"`
	AvgLatencyMS   float64          `json:"avg_latency_ms"`
	PacketLossPct  float64          `json:"packet_loss_pct"`
	CPUTempC       float64          `json:"cpu_temp_c"`
	GPULoadPct     float64          `json:"gpu_load_pct"`
	Timestamp      time.Time        `json:"timestamp"`
	Clusters       []ClusterState   `json:"clusters"`
	CriticalAlerts int              `json:"critical_alerts"`
}

type ClusterState struct {
	ClusterID    string  `json:"cluster_id"`
	LeaderID     int     `json:"leader_id"`
	DroneCount   int     `json:"drone_count"`
	Formation    string  `json:"formation"`
	MissionState string  `json:"mission_state"`
	AvgBattery   float64 `json:"avg_battery"`
}

type MissionPlan struct {
	MissionID   string    `json:"mission_id"`
	Name        string    `json:"name"`
	Formation   string    `json:"formation"`
	ClusterID   string    `json:"cluster_id"`
	Target      [3]float64 `json:"target"`
	Status      string    `json:"status"`
	CreatedAt   time.Time `json:"created_at"`
}

type CommandEnvelope struct {
	CommandID   string                 `json:"command_id"`
	Action      string                 `json:"action"`
	ClusterID   string                 `json:"cluster_id,omitempty"`
	TargetIDs   []int                  `json:"target_ids,omitempty"`
	Payload     map[string]any         `json:"payload"`
	IssuedBy    string                 `json:"issued_by"`
	IssuedAt    time.Time              `json:"issued_at"`
}

type EventRecord struct {
	Type      string         `json:"type"`
	Message   string         `json:"message"`
	Timestamp time.Time      `json:"timestamp"`
	Data      map[string]any `json:"data,omitempty"`
}

type HealthReport struct {
	OnlineDrones     int       `json:"online_drones"`
	TotalDrones      int       `json:"total_drones"`
	CriticalAlerts   int       `json:"critical_alerts"`
	AvgBatteryPct    float64   `json:"avg_battery_pct"`
	MaxCPUTempC      float64   `json:"max_cpu_temp_c"`
	UpdatedAt        time.Time `json:"updated_at"`
}
