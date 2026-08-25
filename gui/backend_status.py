"""Helpers for operator-visible backend mode labeling."""

from __future__ import annotations


def compose_backend_mode(base_mode: str, fallback_reason: str = "") -> str:
    normalized_base = str(base_mode or "simulation").strip().lower() or "simulation"
    normalized_reason = str(fallback_reason or "").strip().lower()
    if not normalized_reason:
        return normalized_base
    if normalized_base == "simulation":
        return f"simulation-fallback:{normalized_reason}"
    return f"{normalized_base}-fallback:{normalized_reason}"


def summarize_localization_data_source(values: list[str]) -> str:
    normalized = [
        str(value or "").strip().lower() for value in values if str(value or "").strip()
    ]
    if "simulation" in normalized:
        return "simulation"
    if "playback" in normalized:
        return "playback"
    if "unavailable" in normalized:
        return "unavailable"
    if "real" in normalized:
        return "real"
    return "unavailable"


def compose_operator_status(
    backend_mode: str,
    localization_data_source: str,
    *,
    simulation_enabled: bool = False,
    real_drone_count: int = 0,
    stale_drone_count: int = 0,
) -> str:
    source = (
        str(localization_data_source or "unavailable").strip().lower() or "unavailable"
    )
    mode = str(backend_mode or "simulation").strip().lower() or "simulation"
    warnings: list[str] = []
    if mode == "edge_swarm":
        warnings.append("EDGE SWARM ACTIVE")
    if simulation_enabled:
        warnings.append("SIMULATION ENABLED")
    if real_drone_count <= 0:
        warnings.append("NO REAL DRONES")
    if stale_drone_count > 0:
        warnings.append(f"STALE TELEMETRY:{stale_drone_count}")
    if source == "simulation":
        warnings.append("LOCALIZATION:SIMULATION")
    elif source == "playback":
        warnings.append("LOCALIZATION:PLAYBACK")
    elif source == "unavailable":
        warnings.append("LOCALIZATION:UNAVAILABLE")
    else:
        warnings.append("LOCALIZATION:REAL")
    return f"Backend: {backend_mode.upper()} | {' | '.join(warnings)}"
