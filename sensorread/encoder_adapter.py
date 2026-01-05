# encoder_adapter.py
from dataclasses import dataclass
from base.data_base import EncoderData

def joint_dict_to_encoder(joint_dict: dict) -> EncoderData:
    """
    Adapter:
    MCU2PC joint dict -> EncoderData
    """
    return EncoderData(
        angle=float(joint_dict.get("knee_angle", 0.0)),
        velocity=float(joint_dict.get("v", 0.0)),
        t_ff=float(joint_dict.get("t_ff", 0.0))
    )
