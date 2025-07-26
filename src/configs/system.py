# src/config/system.py

from pydantic import BaseModel, Field, field_validator


class SystemConfig(BaseModel):
    """Drone configuration parameters"""

    min_target_distance: float = Field(
        default=5.0, ge=1.0, le=50.0, description="Minimum target distance (m)"
    )
    max_target_distance: float = Field(
        default=50.0, ge=10.0, le=200.0, description="Maximum target distance (m)"
    )
    position_range: float = Field(
        default=100.0, ge=20.0, description="Operating area radius from origin (m)"
    )
    random_action_prob: float = Field(
        default=0.3,
        ge=0.0,
        le=1.0,
        description="Base random action probability for exploration",
    )

    reward_scale: float = Field(default=1.0, description="Reward scaling factor")
    episode_steps: int = Field(
        default=1000, ge=50, description="Maximum steps per episode"
    )
    max_episodes: int = Field(
        default=100, ge=1, description="Maximum number of episodes to collect/train"
    )
    action_dim: int = Field(
        default=4,
        description="Action space dimension (e.g., Forward, Right, Down, YawRate)",
    )
    smoothness_penalty_weight: float = Field(
        default=0.05,
        ge=0.0,
        description="Weight for action smoothness penalty in reward",
    )
    no_gps: bool = Field(
        default=True,
        description="Enable operation without GPS (uses alternative estimation)",
    )
    line_navigation: bool = Field(
        default=False,
        description="Enable direct line navigation between start and target.",
    )
    save_interval: int = Field(
        default=5000,
        description="Save model checkpoint every N training steps (concurrent/train mode).",
    )

    sensor_update_frequency: float = Field(
        default=50.0,
        ge=10.0,
        le=100.0,
        description="Expected sensor update frequency (Hz)",
    )
    setpoint_count: int = Field(
        default=10, ge=5, le=30, description="Number of initial setpoints to send"
    )
    setpoint_interval: float = Field(
        default=0.05,
        ge=0.02,
        le=0.2,
        description="Interval between initial setpoints (s)",
    )
    min_episode_steps: int = Field(
        default=10,
        ge=1,
        description="Minimum steps before an episode can terminate successfully via 'done'",
    )

    @field_validator("max_target_distance")
    def max_distance_must_be_greater_than_min(cls, v, info):
        min_dist = info.data.get("min_target_distance")
        if min_dist is not None and v <= min_dist:
            raise ValueError(
                f"Maximum target distance ({v}) must be >= minimum target distance ({min_dist})"
            )
        return v

    @field_validator("min_safe_altitude")
    def safe_altitude_must_be_less_than_target(cls, v, info):
        target_alt = info.data.get("target_altitude")
        if target_alt is not None and v >= target_alt:
            # Allow safe altitude to be equal to target altitude
            if v > target_alt:
                raise ValueError(
                    f"Minimum safe altitude ({v}) must be <= target altitude ({target_alt})"
                )
        return v

    @field_validator("target_altitude")
    def target_alt_must_be_positive(cls, v):
        if v <= 0:
            raise ValueError("Target altitude must be positive")
        return v
