from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from research.ai.common import clamp, rounded
from simulation.vehicle_state import VehicleState


@dataclass(frozen=True)
class WorldKnowledgeSnapshot:
    environment_graph: dict[str, Any]
    dynamic_obstacle_map: list[dict[str, Any]]
    semantic_map: list[dict[str, Any]]
    mission_state_graph: dict[str, Any]
    knowledge_base: dict[str, Any]


class SharedWorldModel:
    def build_snapshot(
        self,
        vehicles: list[VehicleState],
        mission_name: str,
        environment_label: str,
    ) -> WorldKnowledgeSnapshot:
        nodes = [{"id": f"drone-{state.drone_id}", "cluster": state.cluster_id, "role": state.role} for state in vehicles]
        edges = []
        for index, source in enumerate(vehicles):
            for target in vehicles[index + 1 :]:
                edges.append(
                    {
                        "source": f"drone-{source.drone_id}",
                        "target": f"drone-{target.drone_id}",
                        "weight": rounded(clamp((source.sync_confidence + target.sync_confidence) / 2.0, 0.0, 1.0)),
                    }
                )
        obstacle_map = [
            {
                "obstacle_id": f"obs-{state.drone_id}",
                "position": [rounded(state.x + 1.2), rounded(state.y + 0.8), rounded(max(0.0, state.z - 0.4))],
                "severity": rounded(clamp((state.local_obstacle_count + state.shared_obstacle_count) / 8.0, 0.0, 1.0)),
                "source_drone": state.drone_id,
            }
            for state in vehicles
        ]
        semantic_map = [
            {
                "region": state.cluster_id,
                "label": "gps-denied corridor" if state.localization_source != "gps" else "nominal flight lane",
                "confidence": rounded(state.localization_confidence),
            }
            for state in vehicles
        ]
        mission_state_graph = {
            "mission": mission_name,
            "environment": environment_label,
            "states": [
                {"id": "observe", "next": "assign"},
                {"id": "assign", "next": "execute"},
                {"id": "execute", "next": "recover"},
                {"id": "recover", "next": "observe"},
            ],
        }
        knowledge_base = {
            "vehicle_count": len(vehicles),
            "shared_confidence_avg": rounded(sum(state.localization_confidence for state in vehicles) / max(1, len(vehicles))),
            "max_stale_peers": max((state.stale_peer_count for state in vehicles), default=0),
            "mission_health": "stable" if all(state.safety_state == "NORMAL" for state in vehicles) else "degraded",
        }
        return WorldKnowledgeSnapshot(
            environment_graph={"nodes": nodes, "edges": edges, "environment": environment_label},
            dynamic_obstacle_map=obstacle_map,
            semantic_map=semantic_map,
            mission_state_graph=mission_state_graph,
            knowledge_base=knowledge_base,
        )


def build_world_model(vehicles: list[VehicleState], mission_name: str, environment_label: str) -> WorldKnowledgeSnapshot:
    return SharedWorldModel().build_snapshot(vehicles, mission_name, environment_label)
