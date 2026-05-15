// System Designer and Developer: Md Shahanur Islam Shagor
// Project: UVA GPS Denied Navigation in Dynamic Environments
// Technology: C++, Python, Go, CMake

package controlplane

import "time"

type CameraTelemetry struct {
	Status         string  `json:"status,omitempty"`
	FPS            float64 `json:"fps,omitempty"`
	FrameAgeMS     float64 `json:"frame_age_ms,omitempty"`
	Resolution     string  `json:"resolution,omitempty"`
	DroppedFrames  int     `json:"dropped_frames,omitempty"`
	Source         string  `json:"source,omitempty"`
	PreviewURL     string  `json:"preview_url,omitempty"`
	LatestFrameRef string  `json:"latest_frame_ref,omitempty"`
}

type Vector3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type LidarPoint2D struct {
	X         float64 `json:"x"`
	Y         float64 `json:"y"`
	Intensity float64 `json:"intensity,omitempty"`
}

type IMUTelemetry struct {
	Status          string  `json:"status,omitempty"`
	SampleRateHz    float64 `json:"sample_rate_hz,omitempty"`
	LastSampleAgeMS float64 `json:"last_sample_age_ms,omitempty"`
	Accel           Vector3 `json:"accel"`
	Gyro            Vector3 `json:"gyro"`
	Health          string  `json:"health,omitempty"`
	Source          string  `json:"source,omitempty"`
}

type LiDARTelemetry struct {
	Status       string         `json:"status,omitempty"`
	PacketRateHz float64        `json:"packet_rate_hz,omitempty"`
	ScanAgeMS    float64        `json:"scan_age_ms,omitempty"`
	PointCount   int            `json:"point_count,omitempty"`
	Points2D     []LidarPoint2D `json:"points_2d,omitempty"`
	MinRangeM    float64        `json:"min_range_m,omitempty"`
	MaxRangeM    float64        `json:"max_range_m,omitempty"`
	Source       string         `json:"source,omitempty"`
}

type TDOAAnchorTelemetry struct {
	ID         string  `json:"id"`
	X          float64 `json:"x"`
	Y          float64 `json:"y"`
	Z          float64 `json:"z"`
	Visible    bool    `json:"visible"`
	LastSeenMS float64 `json:"last_seen_ms,omitempty"`
}

type TDOATelemetry struct {
	Status             string                `json:"status,omitempty"`
	Source             string                `json:"source,omitempty"`
	VisibleAnchorCount int                   `json:"visible_anchor_count,omitempty"`
	Anchors            []TDOAAnchorTelemetry `json:"anchors,omitempty"`
	EstimatedPosition  Vector3               `json:"estimated_position"`
	CalibrationWarning string                `json:"calibration_warning,omitempty"`
}

type ReplayTelemetry struct {
	Status           string    `json:"status,omitempty"`
	Active           bool      `json:"active,omitempty"`
	FileName         string    `json:"file_name,omitempty"`
	Progress         float64   `json:"progress,omitempty"`
	CurrentTime      float64   `json:"current_time,omitempty"`
	ConfidenceSeries []float64 `json:"confidence_series,omitempty"`
	Source           string    `json:"source,omitempty"`
}

type DroneTelemetry struct {
	DroneID                  int             `json:"drone_id"`
	ClusterID                string          `json:"cluster_id"`
	Role                     string          `json:"role"`
	Source                   string          `json:"source,omitempty"`
	Stale                    bool            `json:"stale,omitempty"`
	Connectivity             string          `json:"connectivity"`
	Reachable                bool            `json:"reachable"`
	Position                 [3]float64      `json:"position"`
	Velocity                 [3]float64      `json:"velocity"`
	AttitudeRPY              [3]float64      `json:"attitude_rpy"`
	ThrustVector             [3]float64      `json:"thrust_vector"`
	CommandedAltitudeM       float64         `json:"commanded_altitude_m,omitempty"`
	CommandedSpeedMPS        float64         `json:"commanded_speed_mps,omitempty"`
	ManualTargetPosition     [3]float64      `json:"manual_target_position,omitempty"`
	ManualControlActive      bool            `json:"manual_control_active,omitempty"`
	DriftM                   float64         `json:"drift_m"`
	BatteryPct               float64         `json:"battery_pct"`
	RSSIDBm                  float64         `json:"rssi_dbm"`
	CPUTempC                 float64         `json:"cpu_temp_c"`
	GPULoadPct               float64         `json:"gpu_load_pct"`
	MissionState             string          `json:"mission_state"`
	LocalizationSource       string          `json:"localization_source,omitempty"`
	LocalizationDataSource   string          `json:"localization_data_source,omitempty"`
	LocalizationState        string          `json:"localization_state,omitempty"`
	LocalizationConfidence   float64         `json:"localization_confidence,omitempty"`
	TDOAConfidence           float64         `json:"tdoa_confidence,omitempty"`
	ConfidenceTrend          float64         `json:"confidence_trend,omitempty"`
	RelocalizationCount      int             `json:"relocalization_count,omitempty"`
	VisibleAnchorCount       int             `json:"visible_anchor_count,omitempty"`
	OccupancyRatio           float64         `json:"occupancy_ratio,omitempty"`
	SyncConfidence           float64         `json:"sync_confidence,omitempty"`
	IMUCameraOffsetMS        float64         `json:"imu_camera_offset_ms,omitempty"`
	PeerCount                int             `json:"peer_count,omitempty"`
	StalePeerCount           int             `json:"stale_peer_count,omitempty"`
	MeshTopologyMode         string          `json:"mesh_topology_mode,omitempty"`
	LocalConsensusState      string          `json:"local_consensus_state,omitempty"`
	LocalConsensusEpoch      uint64          `json:"local_consensus_epoch,omitempty"`
	PeerLatencyMS            float64         `json:"peer_latency_ms,omitempty"`
	MeshBandwidthKBPS        float64         `json:"mesh_bandwidth_kbps,omitempty"`
	EdgeSerializationMode    string          `json:"edge_serialization_mode,omitempty"`
	EdgeAveragePacketBytes   float64         `json:"edge_average_packet_size_bytes,omitempty"`
	EdgeBandwidthSavingsPct  float64         `json:"edge_bandwidth_savings_estimate_pct,omitempty"`
	EdgePacketEncodeLatency  float64         `json:"edge_packet_encode_latency_us,omitempty"`
	AuthMode                 string          `json:"auth_mode,omitempty"`
	AuthFailures             int             `json:"auth_failures,omitempty"`
	UnsignedPackets          int             `json:"unsigned_packets,omitempty"`
	LastAuthResult           string          `json:"last_auth_result,omitempty"`
	PQCReadyStatus           string          `json:"pqc_ready_status,omitempty"`
	DisconnectedOperation    bool            `json:"disconnected_operation,omitempty"`
	EdgeHealthStatus         string          `json:"edge_health_status,omitempty"`
	EdgeAutonomyState        string          `json:"edge_autonomy_state,omitempty"`
	EdgeInferenceStatus      string          `json:"edge_inference_status,omitempty"`
	EdgeInferenceFPS         float64         `json:"edge_inference_fps,omitempty"`
	EdgeInferenceConfidence  float64         `json:"edge_inference_confidence,omitempty"`
	LocalObstacleCount       int             `json:"local_obstacle_count,omitempty"`
	SharedObstacleCount      int             `json:"shared_obstacle_count,omitempty"`
	SecurityState            string          `json:"security_state,omitempty"`
	SecuritySummary          string          `json:"security_summary,omitempty"`
	SecurityTransitionReason string          `json:"security_transition_reason,omitempty"`
	SafetyState              string          `json:"safety_state,omitempty"`
	SafetySummary            string          `json:"safety_summary,omitempty"`
	RemoteCommandAllowed     bool            `json:"remote_command_allowed,omitempty"`
	TelemetryUplinkAllowed   bool            `json:"telemetry_uplink_allowed,omitempty"`
	LinkIntegrityScore       float64         `json:"link_integrity_score,omitempty"`
	TrustEpoch               int             `json:"trust_epoch,omitempty"`
	LastAuthFailureAtS       float64         `json:"last_auth_failure_at_s,omitempty"`
	TamperScore              float64         `json:"tamper_score,omitempty"`
	FirmwareMeasurement      string          `json:"firmware_measurement,omitempty"`
	FirmwareVersion          string          `json:"firmware_version,omitempty"`
	SecureBootState          string          `json:"secure_boot_state,omitempty"`
	BootTrustSummary         string          `json:"boot_trust_summary,omitempty"`
	RollbackCounter          uint64          `json:"rollback_counter,omitempty"`
	MaintenanceMode          bool            `json:"maintenance_mode,omitempty"`
	UpdateChannelState       string          `json:"update_channel_state,omitempty"`
	LastRemoteCommandStatus  string          `json:"last_remote_command_status,omitempty"`
	HealthFlags              []string        `json:"health_flags,omitempty"`
	Camera                   CameraTelemetry `json:"camera,omitempty"`
	IMU                      IMUTelemetry    `json:"imu,omitempty"`
	LiDAR                    LiDARTelemetry  `json:"lidar,omitempty"`
	TDOA                     TDOATelemetry   `json:"tdoa,omitempty"`
	Replay                   ReplayTelemetry `json:"replay,omitempty"`
	Timestamp                time.Time       `json:"timestamp"`
}

type FleetSnapshot struct {
	Drones            []DroneTelemetry `json:"drones"`
	LeaderID          int              `json:"leader_id"`
	AvgLatencyMS      float64          `json:"avg_latency_ms"`
	AvgPeerLatencyMS  float64          `json:"avg_peer_latency_ms"`
	AvgMeshBandwidth  float64          `json:"avg_mesh_bandwidth_kbps"`
	PacketLossPct     float64          `json:"packet_loss_pct"`
	CPUTempC          float64          `json:"cpu_temp_c"`
	GPULoadPct        float64          `json:"gpu_load_pct"`
	Timestamp         time.Time        `json:"timestamp"`
	Clusters          []ClusterState   `json:"clusters"`
	CriticalAlerts    int              `json:"critical_alerts"`
	BackendMode       string           `json:"backend_mode"`
	MeshTopologyMode  string           `json:"mesh_topology_mode"`
	SimulationEnabled bool             `json:"simulation_enabled"`
	RealDroneCount    int              `json:"real_drone_count"`
	StaleDroneCount   int              `json:"stale_drone_count"`
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
	CommandID       string         `json:"command_id"`
	Action          string         `json:"action"`
	ClusterID       string         `json:"cluster_id,omitempty"`
	TargetIDs       []int          `json:"target_ids,omitempty"`
	Payload         map[string]any `json:"payload"`
	IssuedBy        string         `json:"issued_by"`
	IssuerRole      string         `json:"issuer_role,omitempty"`
	IssuedAt        time.Time      `json:"issued_at"`
	ExpiresAt       time.Time      `json:"expires_at,omitempty"`
	Nonce           string         `json:"nonce,omitempty"`
	Authenticated   bool           `json:"authenticated"`
	SecurityProfile string         `json:"security_profile,omitempty"`
	AuditPrevHash   string         `json:"audit_prev_hash,omitempty"`
	AuditHash       string         `json:"audit_hash,omitempty"`
}

type PendingApproval struct {
	ApprovalID    string         `json:"approval_id"`
	Action        string         `json:"action"`
	Payload       map[string]any `json:"payload"`
	PayloadJSON   string         `json:"payload_json"`
	IssuedBy      string         `json:"issued_by"`
	IssuerRole    string         `json:"issuer_role"`
	ApprovedBy    []string       `json:"approved_by,omitempty"`
	CreatedAt     time.Time      `json:"created_at"`
	ExpiresAt     time.Time      `json:"expires_at"`
	ApprovalState string         `json:"approval_state"`
}

type EventRecord struct {
	Type          string         `json:"type"`
	Message       string         `json:"message"`
	Timestamp     time.Time      `json:"timestamp"`
	Data          map[string]any `json:"data,omitempty"`
	AuditPrevHash string         `json:"audit_prev_hash,omitempty"`
	AuditHash     string         `json:"audit_hash,omitempty"`
}

type DeviceRecord struct {
	Identity     string    `json:"identity"`
	DeviceType   string    `json:"device_type"`
	ClusterScope []string  `json:"cluster_scope,omitempty"`
	Status       string    `json:"status,omitempty"`
	NotAfter     time.Time `json:"not_after,omitempty"`
}

type HealthReport struct {
	OnlineDrones      int       `json:"online_drones"`
	TotalDrones       int       `json:"total_drones"`
	CriticalAlerts    int       `json:"critical_alerts"`
	AvgBatteryPct     float64   `json:"avg_battery_pct"`
	MaxCPUTempC       float64   `json:"max_cpu_temp_c"`
	UpdatedAt         time.Time `json:"updated_at"`
	BackendMode       string    `json:"backend_mode"`
	MeshTopologyMode  string    `json:"mesh_topology_mode"`
	SimulationEnabled bool      `json:"simulation_enabled"`
	RealDroneCount    int       `json:"real_drone_count"`
	StaleDroneCount   int       `json:"stale_drone_count"`
}
