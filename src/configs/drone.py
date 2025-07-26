# src/cfg/model_config.py

from typing import Optional
import torch
from pydantic import BaseModel, Field, field_validator


class GimbalConfig(BaseModel):
    """Configuration parameters specific to the Gimbal Controller."""

    # --- 行為參數 ---
    gimbal_id: int = Field(default=0, description="Gimbal ID")
    initial_mode: str = Field(default="YAW_FOLLOW", description="Gimbal initial_mode")
    update_interval: float = Field(
        default=0.2,
        ge=0.01,
        le=1.0,
        description="Interval for internal state updates and telemetry checks (seconds)",
    )
    control_loop_frequency: float = Field(
        default=10.0,
        ge=1.0,
        le=50.0,
        description="Frequency of the internal P-control loop for angle commands (Hz)",
    )

    pitch_limit_min: float = Field(
        default=-90.0, le=0, description="Minimum pitch angle min limit (degrees)"
    )
    pitch_limit_max: float = Field(
        default=30.0, ge=0, description="Maximum pitch angle max limit (degrees)"
    )
    roll_limit_min: float = Field(
        default=-45.0, le=0, description="Minimum roll angle min limit (degrees)"
    )
    roll_limit_max: float = Field(
        default=45.0, ge=0, description="Maximum roll angle max limit (degrees)"
    )
    yaw_limit_min: float = Field(
        default=-180.0, le=0, description="Minimum yaw angle min limit (degrees)"
    )
    yaw_limit_max: float = Field(
        default=180.0, ge=0, description="Maximum yaw angle max limit (degrees)"
    )
    max_rate: float = Field(
        default=90.0,
        ge=10.0,
        description="Maximum angular rate for gimbal movement (degrees/second)",
    )

    # --- P 控制器參數 (用於角度控制) ---
    kp_pitch: float = Field(
        default=1.8,
        ge=0.0,
        description="Proportional gain (Kp) for pitch control loop (needs tuning)",
    )
    kp_yaw: float = Field(
        default=1.8,
        ge=0.0,
        description="Proportional gain (Kp) for yaw control loop (needs tuning)",
    )
    angle_tolerance: float = Field(
        default=1.0,
        ge=0.1,
        le=5.0,
        description="Angle tolerance for P-control loop target reaching (degrees)",
    )

    # --- 進階功能參數 ---
    drone_state_history_len: int = Field(
        default=64, ge=5, description="Length of drone state history for prediction"
    )
    smoothing_min: float = Field(
        default=0.2,
        ge=0.0,
        le=1.0,
        description="Minimum smoothing factor for adaptive gimbal control",
    )
    smoothing_max: float = Field(
        default=0.9,
        ge=0.0,
        le=1.0,
        description="Maximum smoothing factor for adaptive gimbal control",
    )
    smoothing_velocity_threshold: float = Field(
        default=5.0,
        ge=0.1,
        description="Drone velocity threshold for adaptive smoothing adjustment (m/s)",
    )
    prediction_horizon: float = Field(
        default=0.3,
        ge=0.0,
        le=2.0,
        description="Prediction horizon for predictive target tracking (seconds)",
    )
    prediction_accel_weight: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="Weighting factor for acceleration term in prediction",
    )

    # --- 驗證器 ---
    @field_validator("pitch_limit_max")
    def check_pitch_limits(cls, v, info):
        min_val = info.data.get("pitch_limit_min")
        if min_val is not None and v < min_val:
            raise ValueError(
                f"Max pitch limit ({v}) must be >= min pitch limit ({min_val})"
            )
        return v

    @field_validator("roll_limit_max")
    def check_roll_limits(cls, v, info):
        min_val = info.data.get("roll_limit_min")
        if min_val is not None and v < min_val:
            raise ValueError(
                f"Max roll limit ({v}) must be >= min roll limit ({min_val})"
            )
        return v

    @field_validator("yaw_limit_max")
    def check_yaw_limits(cls, v, info):
        if not (-360 <= v <= 360):
            raise ValueError(f"Yaw limit max ({v}) out of reasonable range")
        return v

    @field_validator("smoothing_max")
    def check_smoothing_limits(cls, v, info):
        min_val = info.data.get("smoothing_min")
        if min_val is not None and v < min_val:
            raise ValueError(
                f"Max smoothing ({v}) must be >= min smoothing ({min_val})"
            )
        return v


class DroneConfig(BaseModel):
    """Drone configuration parameters"""

    manual: bool = Field(default=True, description="Manual control mode")
    model_path: Optional[str] = Field(default=None, description="AI model file path")
    device: str = Field(
        default="cuda" if torch.cuda.is_available() else "cpu",
        description="AI computation device",
    )
    system_address: str = Field(
        default="udp://:14540", description="MAVSDK connection address"
    )
    connection_timeout: int = Field(
        default=30, ge=5, le=120, description="Connection timeout seconds"
    )
    imu_timeout: float = Field(
        default=0.5,
        ge=0.1,
        le=2.0,
        description="Time threshold before IMU data is considered stale (s)",
    )
    max_velocity: float = Field(
        default=5.0, ge=1.0, le=15.0, description="Maximum velocity (m/s)"
    )
    max_yaw_rate: float = Field(
        default=90.0, ge=10.0, le=180.0, description="Maximum yaw rate (deg/s)"
    )
    velocity_increment: float = Field(
        default=0.5,
        ge=0.1,
        le=2.0,
        description="Velocity increment for smoothing/manual",
    )
    use_ai_controller: bool = Field(
        default=False, description="Whether to use AI controller primarily"
    )
    sensor_dim: int = Field(
        default=15, ge=6, description="Sensor input vector dimension"
    )
    sequence_length: int = Field(
        default=64, ge=8, le=128, description="Sensor history sequence length"
    )
    ai_control_frequency: float = Field(
        default=10.0, ge=1.0, le=30.0, description="AI control loop frequency (Hz)"
    )
    min_safe_altitude: float = Field(
        default=1.5,
        ge=0.5,
        le=5.0,
        description="Minimum safe altitude above ground (m)",
    )
    target_altitude: float = Field(
        default=5.0, ge=1.0, le=20.0, description="Target flight altitude (m)"
    )
    altitude_tolerance: float = Field(
        default=1.0,
        ge=0.2,
        le=5.0,
        description="Tolerance for reaching target altitude (m)",
    )
    decay_factor: float = Field(0.9, description="Control decay factor")
    zero_threshold: float = Field(0.05, description="Threshold to zero out values")
    navigate_by_camera: bool = Field(
        default=False,
        description="Use camera direction as forward for navigation commands instead of drone body",
    )
    use_gimbal: bool = Field(
        default=True, description="Enable gimbal integration if available"
    )
    gimbal: Optional[GimbalConfig] = Field(
        default_factory=GimbalConfig, description="Gimbal specific configuration"
    )
