# Limitations

Date: July 17, 2026

## Supported Scope

The supported scope of this repository is software research evidence.

## Unsupported Claims

No claim is made regarding:

- physical flight validation
- PX4 integration
- Gazebo validation
- SITL validation
- ROS2 validation
- hardware qualification
- aircraft certification
- operational fleet deployment in the field
- peer-reviewed publication acceptance

## Technical Limitations

- many timing measurements are workstation-local and not normalized across platforms
- simulation, software HIL, and digital twin layers are abstractions rather than hardware-coupled proofs
- AI modules in later phases rely on deterministic or model-agnostic interfaces rather than trained field models
- Linux support is represented by tracked workflows and earlier evidence; the Phase 12 rerun itself was conducted from Windows
- compiler executables were not directly exposed on the active shell during Phase 12 even though the native validation build artifacts were usable

## Archive Limitation

The official DOI is an archive identifier for the software artifact. It is not evidence of external review outcome or venue acceptance.
