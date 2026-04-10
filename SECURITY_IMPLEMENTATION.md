# Military-Grade Layered Hardening Design

This document defines a practical high-assurance security architecture for the `drone_swarm` project.

It is not based on "perfect security". It is based on layered trust, strong cryptography, narrow authority,
tamper resistance, rapid detection, and safe failure modes.

## 1. Security Goals

Primary goals:

- prevent unauthorized command injection
- prevent drone-to-drone spoofing
- prevent replay and session hijacking
- reduce blast radius if one device is compromised
- ensure a compromised link does not immediately become a flight-control compromise
- force the drone into safe autonomy when trust is degraded
- make every high-risk action auditable

Non-goals:

- "impossible to hack"
- trusting a single shared secret for the whole swarm
- trusting the dashboard as a fully privileged endpoint

## 2. Current Security Posture In This Repo

The repo already has a useful starting point:

- the C++ swarm layer has a secure-envelope implementation with HMAC, signatures, replay rejection, and ledger chaining concepts in `SwarmSecurity`
- the drone runtime enables swarm security when `DRONE_SWARM_SECRET` is present
- the autonomy layer already supports degraded and recovery behaviors

Current architectural weaknesses:

- swarm trust is rooted in a shared secret instead of per-device identity
- the Go control plane HTTP API has no authentication, authorization, or TLS
- dashboard-to-backend command submission is unauthenticated
- there is no certificate lifecycle, revocation, or role separation
- there is no signed firmware / secure boot flow in the repo
- there is no operator approval policy for critical commands
- there is no formal security-state machine tied to flight behavior

## 3. Threat Model

Assume these attacker types:

1. Radio/network attacker
   Can sniff, replay, inject, and spoof packets on local links.

2. Backend attacker
   Can reach the control-plane API and attempt command injection.

3. Rogue operator endpoint
   Can use a stolen laptop or dashboard session.

4. Compromised drone
   One drone's software or keys are stolen.

5. Supply-chain / maintenance attacker
   Tries to push malicious firmware, config, or updates.

6. Physical attacker
   Gets temporary access to a drone and tries to extract secrets.

## 4. Layered Security Architecture

### Layer A. Hardware Root of Trust

Every drone should have:

- secure boot enabled
- signed bootloader
- signed firmware image
- anti-rollback version counter
- per-device private key in secure element / TPM / TrustZone-backed store
- debug ports disabled or maintenance-gated

Recommended trust anchors:

- drone device identity keypair
- firmware signing root
- backend CA root
- operator CA root

If secure hardware is unavailable in the short term:

- use OS-protected key storage
- encrypt at-rest secrets
- require physical maintenance mode for reflashing

### Layer B. Per-Device Identity

Replace the single swarm shared secret with per-device credentials.

Each entity gets a unique identity:

- each drone
- each control-plane instance
- each operator console
- each provisioning station

Use:

- `X25519` or platform TLS identity for key exchange
- `Ed25519` or platform certificate private keys for signatures
- short-lived certificates signed by a project CA

Minimum certificate fields:

- subject ID
- role: `drone`, `control-plane`, `operator`, `provisioner`
- allowed cluster / fleet scope
- expiry
- firmware trust profile

### Layer C. mTLS Everywhere Offboard

All offboard network paths must require mutual TLS:

- dashboard <-> control plane
- drone telemetry uplink <-> control plane
- provisioning tool <-> drone
- maintenance service <-> drone

Rules:

- TLS 1.3 only
- no plaintext HTTP in production
- certificate pinning for drone-side backend trust
- reject self-signed or test certs outside lab mode

### Layer D. Signed Command Protocol

Every flight-affecting command must be signed end-to-end.

Each command envelope should include:

- `command_id`
- `issued_at`
- `expires_at`
- `nonce`
- `issuer_id`
- `issuer_role`
- `target_scope`
- `action`
- `payload`
- `approval_context`
- `signature`

Validation rules on drone:

- signature must verify against trusted operator/control-plane certificate chain
- issuer role must be authorized for that command
- command must not be expired
- nonce must be unused
- sequence / monotonic counter must be newer than prior session state
- target scope must match this drone and cluster
- safety policy must allow the requested command

Critical commands should require dual authorization:

- emergency disarm
- geofence override
- firmware update
- mission profile replacement
- formation leader reassignment during active flight

### Layer E. Drone-to-Drone Secure Mesh

Inter-drone traffic should move from a shared-secret design to authenticated sessions.

Target model:

- each drone signs its ephemeral session key
- peers verify session keys with device certificates
- peers derive per-session symmetric AEAD keys
- all swarm packets use authenticated encryption

Recommended packet security:

- `ChaCha20-Poly1305` or `AES-256-GCM`
- per-peer session key
- rotating session epoch
- nonce window / replay bitmap
- strict packet freshness bounds

Ledger chaining is still useful, but only as an integrity supplement, not as the primary trust anchor.

### Layer F. Zero-Trust Authorization

Do not treat "authenticated" as "fully trusted".

Authority model:

- operator can request commands
- control plane can validate policy and fan out
- drone makes the final safety decision

Enforce:

- RBAC for operator actions
- cluster scoping
- mission scoping
- environment scoping such as lab vs field vs production
- two-person approval for critical actions

Example:

- a dashboard operator may request `formation_change`
- only a mission commander role may request `leader_elect`
- only maintenance role may request `firmware_update`
- no remote operator may bypass geofence without dual approval

### Layer G. Safety Gate On The Drone

The drone must remain the final policy enforcement point.

Before accepting external commands, the drone should check:

- current localization confidence
- battery reserve
- health state
- geofence rules
- no-fly mission lock
- swarm consistency state
- command freshness
- issuer trust level

If checks fail:

- reject the command
- record a signed audit event
- enter a safe local behavior if trust degradation is severe

### Layer H. Security State Machine

Add an explicit security state machine to the drone runtime:

- `TRUSTED`
- `DEGRADED_LINK`
- `AUTH_SUSPECT`
- `PEER_SPOOF_SUSPECT`
- `COMMAND_REPLAY_SUSPECT`
- `CONTROL_PLANE_UNTRUSTED`
- `ISOLATED_AUTONOMY`
- `SAFE_RETURN`
- `LAND_IMMEDIATELY`

Example transitions:

- repeated nonce failures -> `COMMAND_REPLAY_SUSPECT`
- invalid cert chain from backend -> `CONTROL_PLANE_UNTRUSTED`
- mesh peer identity mismatch -> `PEER_SPOOF_SUSPECT`
- multiple failed high-risk commands -> `AUTH_SUSPECT`

Each state should map to flight behavior:

- ignore remote commands
- keep telemetry one-way only
- maintain formation locally
- return home
- hold and await re-authentication
- land immediately if safety margin collapses

### Layer I. Update Security

Firmware and software updates must be:

- signed offline by trusted signing keys
- versioned
- anti-rollback protected
- hash verified on-device
- logged to immutable audit trail

Rules:

- no unsigned OTA
- no direct install from dashboard upload
- no production update from a generic operator role

### Layer J. Observability And Tamper Detection

Log and alert on:

- failed certificate validation
- repeated nonce reuse
- replay rejections
- command bursts
- cluster-scope mismatch
- impossible role changes
- firmware hash mismatch
- frequent session renegotiation
- backend identity changes
- suspicious geofence override attempts

Keep separate logs for:

- flight events
- security events
- command approvals
- firmware / maintenance actions

## 5. Protocol Design For This Repo

### 5.1 Dashboard -> Control Plane

Current state:

- plain HTTP JSON command submission

Target:

- mTLS
- signed command object from operator workstation
- backend verifies signature and authorization
- backend attaches policy decision and forwards command envelope

### 5.2 Control Plane -> Drone

Target envelope:

```text
{
  envelope_version,
  command_id,
  action,
  payload,
  issued_at,
  expires_at,
  nonce,
  issuer_id,
  issuer_role,
  target_ids,
  cluster_id,
  policy_hash,
  approval_count,
  operator_signature,
  control_plane_signature
}
```

Validation on drone:

- verify both signatures
- verify policy hash if cached policy is present
- verify nonce freshness
- verify TTL
- verify target scope
- verify action allowed in current safety state

### 5.3 Drone -> Control Plane Telemetry

Telemetry should be:

- mTLS protected
- optionally signed at message level
- tagged with monotonic sequence number
- tagged with local security state

Add fields:

- `security_state`
- `trust_epoch`
- `last_auth_failure_at`
- `link_integrity_score`
- `firmware_measurement`

### 5.4 Drone -> Drone Mesh

Keep the existing secure-envelope concept, but evolve it:

- replace shared secret derivation with per-peer authenticated session setup
- replace CBC + HMAC construction with AEAD
- keep replay rejection, sequence windows, and chained ledger semantics for critical mission events

## 6. Repo-Specific Hardening Plan

### Phase 1. Immediate hardening

- remove dev-secret fallback in `src/main.cpp`
- fail startup in production mode if swarm secret or device cert is missing
- add HTTPS and mTLS support to the Go control plane
- add authentication and authorization middleware for command routes
- add signed-command schema to dashboard and backend
- add nonce / expiry / issuer fields to command handling
- separate lab mode from production mode using an explicit runtime profile

### Phase 2. Device identity and policy

- introduce per-drone certificates and device registry
- add certificate revocation and rotation
- add RBAC roles for dashboard users
- add critical-command approval workflow
- add command audit chain

### Phase 3. Onboard enforcement

- add explicit security-state machine to the C++ runtime
- gate external commands through safety + authorization policy checks
- bind failsafe behavior to security-state transitions
- add tamper / trust status to telemetry

### Phase 4. Boot and update trust

- signed firmware validation
- secure boot integration
- anti-rollback counters
- secure maintenance workflow

## 7. Concrete Weak Points To Fix First

1. Shared secret model

The current swarm security context derives peer material from one shared secret. If that secret leaks from one drone,
the whole swarm trust boundary is weakened.

2. Unauthenticated control-plane API

The Go server currently accepts telemetry and commands without auth. This is the most urgent network-side gap.

3. Dashboard trust is too broad

The dashboard can issue control-plane commands without operator identity proof, approval policy, or signature.

4. Dev fallback secret

The runtime currently falls back to a development secret if `DRONE_SWARM_SECRET` is absent. That should never happen in
production mode.

## 8. Recommended Cryptographic Choices

Use platform-standard or audited libraries only.

Recommended defaults:

- transport: `TLS 1.3`
- command signature: `Ed25519`
- session key exchange: `X25519`
- packet encryption: `ChaCha20-Poly1305` or `AES-256-GCM`
- KDF: `HKDF-SHA256`
- password-based setup only for lab bootstrap, not production trust

Avoid:

- custom crypto formats unless strictly necessary
- long-lived shared fleet secrets as the primary trust anchor
- CBC + separate MAC for new protocol design when AEAD is available

## 9. Production Profiles

Define three explicit modes:

### `lab`

- allows simulation
- allows local development certificates
- allows reduced controls
- logs warnings loudly

### `field`

- requires mTLS
- requires signed commands
- requires device registry checks
- limited maintenance access

### `production`

- secure boot required
- signed updates only
- no dev secrets
- no plaintext APIs
- dual approval for critical actions
- strict certificate rotation and revocation

## 10. Success Criteria

The architecture is working when:

- a spoofed dashboard cannot issue commands
- a replayed command is rejected
- a rogue peer drone cannot join the mesh
- stealing one drone's keys does not give fleet-wide trust
- loss of backend trust pushes drones into safe autonomy
- every critical command is attributable to a specific approved identity

## 11. Suggested Next Implementation Work

Best next coding steps in this repo:

1. add authn/authz middleware and TLS configuration to the Go control plane
2. define a signed command envelope shared by Python dashboard and Go backend
3. remove production use of shared-secret fallback in the C++ runtime
4. add a drone security-state enum and enforce command gating in the onboard runtime
5. evolve `SwarmSecurity` from shared-secret derivation to per-device authenticated sessions
