"""Modular spacecraft and mission specifications for Project 7."""
from .client_spec import (
    ClientPerceptionSpec,
    ClientSpec,
    RingFeature,
    WingSpec,
    client_double_wing,
    client_p7,
)
from .mission_config import MissionConfig
from .servicer_spec import (
    ArmSpec,
    BusControlSpec,
    SensorRigSpec,
    ServicerSpec,
    ToolSpec,
    servicer_rev2,
)

__all__ = [
    "ArmSpec",
    "BusControlSpec",
    "ClientPerceptionSpec",
    "ClientSpec",
    "MissionConfig",
    "RingFeature",
    "SensorRigSpec",
    "ServicerSpec",
    "ToolSpec",
    "WingSpec",
    "client_double_wing",
    "client_p7",
    "servicer_rev2",
]
