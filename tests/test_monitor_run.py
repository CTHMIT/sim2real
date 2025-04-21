import pytest
import asyncio
from unittest.mock import MagicMock, patch, AsyncMock
from monitor_run import (
    DroneConfig,
    ControlState,
    connect_drone,
    check_drone_health,
    arm_drone,
    takeoff_drone,
    check_gimbal_devices,
    DroneControlGUI,
)


class TestDroneConfig:
    def test_config_defaults(self):
        config = DroneConfig()
        assert config.system_address.startswith("udp://")
        assert config.default_takeoff_altitude > 0
        assert config.speed_increment > 0
        assert config.max_speed > 0
        assert config.acceleration_factor > 0
        assert 0 < config.decay_factor < 1
        assert config.zero_threshold > 0

    def test_config_customization(self):
        custom_config = DroneConfig(
            system_address="serial:///dev/ttyACM0",
            default_takeoff_altitude=10.0,
            max_speed=5.0,
        )
        assert custom_config.system_address == "serial:///dev/ttyACM0"
        assert custom_config.default_takeoff_altitude == 10.0
        assert custom_config.max_speed == 5.0
        assert custom_config.speed_increment == 0.5


class TestControlState:
    def test_control_state_defaults(self):
        state = ControlState()
        assert state.velocity_forward == 0.0
        assert state.velocity_right == 0.0
        assert state.velocity_down == 0.0
        assert state.yawspeed == 0.0
        assert state.gimbal_pitch_rate == 0.0
        assert state.gimbal_yaw_rate == 0.0
        assert not state.exit_flag

    def test_control_state_customization(self):
        state = ControlState(velocity_forward=1.0, velocity_down=-0.5, exit_flag=True)
        assert state.velocity_forward == 1.0
        assert state.velocity_right == 0.0
        assert state.velocity_down == -0.5
        assert state.yawspeed == 0.0
        assert state.exit_flag is True


class TestControlStateManipulation:
    def test_process_keys_forward(self):
        config = DroneConfig()
        state = ControlState()
        gui = DroneControlGUI(MagicMock(), config, state)

        gui.pressed_keys.add("w")
        gui.process_keys()

        assert state.velocity_forward > 0
        assert state.velocity_right == 0
        assert state.velocity_down == 0

    def test_process_keys_multiple(self):
        config = DroneConfig()
        state = ControlState()
        gui = DroneControlGUI(MagicMock(), config, state)

        gui.pressed_keys.add("w")  # Forward
        gui.pressed_keys.add("d")  # Right
        gui.pressed_keys.add("q")  # Yaw left
        gui.process_keys()

        assert state.velocity_forward > 0
        assert state.velocity_right > 0
        assert state.yawspeed < 0

    def test_velocity_decay(self):
        config = DroneConfig(decay_factor=0.5, zero_threshold=0.01)
        state = ControlState(velocity_forward=1.0)
        gui = DroneControlGUI(MagicMock(), config, state)
        gui.process_keys()

        assert 0 < state.velocity_forward < 1.0

        initial_velocity = state.velocity_forward

        gui.process_keys()

        assert state.velocity_forward < initial_velocity

        for _ in range(15):
            gui.process_keys()

        assert state.velocity_forward == 0.0

    def test_exit_control(self):
        config = DroneConfig()
        state = ControlState()
        root_mock = MagicMock()
        root_mock.winfo_exists.return_value = True

        gui = DroneControlGUI(root_mock, config, state)
        gui.exit_control()

        assert state.exit_flag is True
        root_mock.destroy.assert_called_once()


class AsyncIterator:
    def __init__(self, items):
        self.items = items

    def __aiter__(self):
        return self

    async def __anext__(self):
        if not self.items:
            raise StopAsyncIteration
        return self.items.pop(0)


@pytest.mark.asyncio
class TestDroneConnect:
    async def test_connect_drone_success(self):

        async def mock_connection_state():
            connected_state = MagicMock()
            connected_state.is_connected = True
            yield connected_state

        mock_drone = AsyncMock()

        mock_drone.core.connection_state = mock_connection_state

        with patch("monitor_run.System", return_value=mock_drone):
            config = DroneConfig()
            result = await connect_drone(config)

            assert result is mock_drone

    async def test_connect_drone_timeout(self):
        mock_drone = AsyncMock()
        mock_drone.connect.side_effect = asyncio.TimeoutError()

        async def mock_connection_state():
            state = MagicMock()
            state.is_connected = False
            yield state

        mock_drone.core.connection_state = mock_connection_state

        with patch("monitor_run.System", return_value=mock_drone):
            config = DroneConfig()
            result = await connect_drone(config)

            assert result is None


@pytest.mark.asyncio
class TestDroneHealth:
    async def test_health_check_success(self):
        async def mock_health_generator():
            mock_health = MagicMock()
            mock_health.is_gyrometer_calibration_ok = True
            mock_health.is_accelerometer_calibration_ok = True
            mock_health.is_magnetometer_calibration_ok = True
            mock_health.is_armable = True
            yield mock_health

        mock_drone = AsyncMock()

        mock_drone.telemetry.health = mock_health_generator

        result = await check_drone_health(mock_drone)

        assert result is True

    async def test_health_check_partial_success(self):
        async def mock_health_generator():
            mock_health = MagicMock()
            mock_health.is_gyrometer_calibration_ok = True
            mock_health.is_accelerometer_calibration_ok = False
            mock_health.is_magnetometer_calibration_ok = True
            mock_health.is_armable = False
            yield mock_health

        mock_drone = AsyncMock()

        mock_drone.telemetry.health = mock_health_generator

        result = await check_drone_health(mock_drone)

        assert result is True


@pytest.mark.asyncio
class TestTakeoffLand:
    async def test_arm_drone_success(self):
        mock_drone = AsyncMock()
        result = await arm_drone(mock_drone)
        mock_drone.action.arm.assert_called_once()
        assert result is True

    async def test_takeoff_drone_success(self, monkeypatch):
        async def mock_position_generator():
            mock_position = MagicMock()
            mock_position.relative_altitude_m = 5.0
            yield mock_position

        mock_drone = AsyncMock()
        config = DroneConfig(default_takeoff_altitude=5.0)

        mock_drone.telemetry.position = mock_position_generator

        result = await takeoff_drone(mock_drone, config)

        mock_drone.action.set_takeoff_altitude.assert_called_once_with(5.0)
        mock_drone.action.takeoff.assert_called_once()
        assert result is True

    async def test_check_gimbal_devices(self, monkeypatch):
        async def mock_gimbal_list():
            mock_gimbal = MagicMock()
            mock_gimbal.gimbal_id = 1
            mock_gimbal.model_name = "Test Gimbal"
            mock_gimbal.vendor_name = "Test Vendor"

            mock_gimbal_list = MagicMock()
            mock_gimbal_list.gimbals = [mock_gimbal]

            yield mock_gimbal_list

        mock_drone = AsyncMock()
        config = DroneConfig()

        mock_drone.gimbal.gimbal_list = mock_gimbal_list

        await check_gimbal_devices(mock_drone, config)

        assert config.target_gimbal_id == 1
