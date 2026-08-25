# Configuration Reference

## Schema-backed files

Phase 5 adds machine-readable schemas under `config/schema/` for these tracked files:

- `config/runtime.json`
- `config/runtime.example.json`
- `config/anchors.json`
- `config/anchors.example.json`
- `config/lidar.json`
- `config/lidar.example.json`
- `config/detector_labels.json`
- `config/detector_labels.example.json`
- `config/swarm_edge_protocol.json`
- `config/swarm_edge_protocol.example.json`

## Intent

These schemas document published configuration shape for reproducibility and release validation.

They do not imply that every documented field is consumed by the current runtime in every code path.

## Environment-driven control-plane settings

Key deployment variables:

- `DRONE_SWARM_ADDR`
- `DRONE_BACKEND_MODE`
- `DRONE_BACKEND_SIMULATION_ENABLED`
- `DRONE_BACKEND_STALE_SEC`
- `DRONE_BACKEND_DEMO_FLEET_SIZE`
- `DRONE_SECURITY_PROFILE`
- `DRONE_REQUIRE_SIGNED_COMMANDS`
- `DRONE_TLS_ENABLED`
- `DRONE_TLS_REQUIRE_CLIENT_CERT`
- `DRONE_TLS_CERT_FILE`
- `DRONE_TLS_KEY_FILE`
- `DRONE_TLS_CA_FILE`
- `DRONE_OPERATOR_ID`
- `DRONE_OPERATOR_ROLE`
- `DRONE_OPERATOR_SECRET`
- `DRONE_OPERATOR_CREDENTIALS`
- `DRONE_DEVICE_REGISTRY`
- `DRONE_DEVICE_REGISTRY_FILE`

## Secret-bearing variables

Do not track values for:

- `DRONE_OPERATOR_SECRET`
- `DRONE_OPERATOR_CREDENTIALS`
- `DRONE_SWARM_SECRET`
- any TLS private key path that points to local secret material

## Validation

Run:

```powershell
python scripts/validate_config_schemas.py
```

The release-validation workflow also runs this check automatically.

