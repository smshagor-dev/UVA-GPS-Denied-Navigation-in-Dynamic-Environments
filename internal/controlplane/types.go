// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

package controlplane

import "time"

type DroneTelemetry struct {
	DroneID                int        `json:"drone_id"`
	ClusterID              string     `json:"cluster_id"`
	Role                   string     `json:"role"`
	Connectivity           string     `json:"connectivity"`
	Reachable              bool       `json:"reachable"`
	Position               [3]float64 `json:"position"`
	Velocity               [3]float64 `json:"velocity"`
	AttitudeRPY            [3]float64 `json:"attitude_rpy"`
	ThrustVector           [3]float64 `json:"thrust_vector"`
	CommandedAltitudeM     float64    `json:"commanded_altitude_m,omitempty"`
	CommandedSpeedMPS      float64    `json:"commanded_speed_mps,omitempty"`
	DriftM                 float64    `json:"drift_m"`
	BatteryPct             float64    `json:"battery_pct"`
	RSSIDBm                float64    `json:"rssi_dbm"`
	CPUTempC               float64    `json:"cpu_temp_c"`
	GPULoadPct             float64    `json:"gpu_load_pct"`
	MissionState           string     `json:"mission_state"`
	LocalizationSource     string     `json:"localization_source,omitempty"`
	LocalizationState      string     `json:"localization_state,omitempty"`
	LocalizationConfidence float64    `json:"localization_confidence,omitempty"`
	TDOAConfidence         float64    `json:"tdoa_confidence,omitempty"`
	ConfidenceTrend        float64    `json:"confidence_trend,omitempty"`
	RelocalizationCount    int        `json:"relocalization_count,omitempty"`
	VisibleAnchorCount     int        `json:"visible_anchor_count,omitempty"`
	OccupancyRatio         float64    `json:"occupancy_ratio,omitempty"`
	SyncConfidence         float64    `json:"sync_confidence,omitempty"`
	IMUCameraOffsetMS      float64    `json:"imu_camera_offset_ms,omitempty"`
	HealthFlags            []string   `json:"health_flags,omitempty"`
	Timestamp              time.Time  `json:"timestamp"`
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
	MissionID string     `json:"mission_id"`
	Name      string     `json:"name"`
	Formation string     `json:"formation"`
	ClusterID string     `json:"cluster_id"`
	Target    [3]float64 `json:"target"`
	Status    string     `json:"status"`
	CreatedAt time.Time  `json:"created_at"`
}

type CommandEnvelope struct {
	CommandID string         `json:"command_id"`
	Action    string         `json:"action"`
	ClusterID string         `json:"cluster_id,omitempty"`
	TargetIDs []int          `json:"target_ids,omitempty"`
	Payload   map[string]any `json:"payload"`
	IssuedBy  string         `json:"issued_by"`
	IssuedAt  time.Time      `json:"issued_at"`
}

type EventRecord struct {
	Type      string         `json:"type"`
	Message   string         `json:"message"`
	Timestamp time.Time      `json:"timestamp"`
	Data      map[string]any `json:"data,omitempty"`
}

type HealthReport struct {
	OnlineDrones   int       `json:"online_drones"`
	TotalDrones    int       `json:"total_drones"`
	CriticalAlerts int       `json:"critical_alerts"`
	AvgBatteryPct  float64   `json:"avg_battery_pct"`
	MaxCPUTempC    float64   `json:"max_cpu_temp_c"`
	UpdatedAt      time.Time `json:"updated_at"`
}
