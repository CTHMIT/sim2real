# src/cfg/model_config.py

from typing import Optional, Literal
import torch
from pydantic import BaseModel, Field, field_validator


class ModelConfig(BaseModel):
    """Configuration for Model and Training"""

    device: str = Field(default="cuda" if torch.cuda.is_available() else "cpu")
    model_path: Optional[str] = Field(default=None)
    buffer_path: Optional[str] = Field(default=None)

    # Model Arch
    sensor_dim: int = Field(default=15)
    state_dim: int = Field(default=64)
    action_dim: int = Field(default=4)
    hidden_dim: int = Field(default=128)
    sequence_length: int = Field(default=64)
    num_attention_heads: int = Field(default=4)
    num_attention_layers: int = Field(default=2)
    max_relative_position: int = Field(default=16)
    activation: Literal["relu", "tanh", "leaky_relu", "gelu"] = Field(default="gelu")
    dropout: float = Field(default=0.1)

    # Training Hyperparams
    learning_rate: float = Field(default=3e-4)
    batch_size: int = Field(default=256)
    buffer_capacity: int = Field(default=500000)
    gamma: float = Field(default=0.99)
    tau: float = Field(default=0.005)
    initial_alpha: float = Field(default=0.2)
    automatic_entropy_tuning: bool = Field(default=True)
    weight_decay: float = Field(default=0.0001)
    grad_clip: float = Field(default=1.0)

    target_update_interval: int = Field(default=2)
    save_interval: int = Field(default=5000)
    log_interval: int = Field(default=100)

    @field_validator("device")
    def validate_device(cls, v):
        if v.startswith("cuda") and not torch.cuda.is_available():
            return "cpu"
        return v

    class Config:
        arbitrary_types_allowed = True
