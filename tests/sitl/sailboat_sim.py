"""
Sailboat Simulator - Python implementation matching C++ SIM_Sailboat_USV

This module provides a pure Python implementation of the sailboat physics
simulation for testing without requiring C++ compilation.
"""

import math
from enum import IntEnum
from dataclasses import dataclass, field
from typing import Optional, Tuple, List
import time


class WingsailType(IntEnum):
    """Wingsail control type enumeration"""
    WINGSAIL_ROTATION = 0  # Rudder + wingsail rotation
    WINGSAIL_FLAP = 1      # Rudder + flap control
    WINGSAIL_FREE = 2      # Rudder only, self-balancing


class BeaufortScale(IntEnum):
    """Beaufort wind scale"""
    CALM = 0
    LIGHT_AIR = 1
    LIGHT_BREEZE = 2
    GENTLE_BREEZE = 3
    MODERATE_BREEZE = 4
    FRESH_BREEZE = 5
    STRONG_BREEZE = 6
    NEAR_GALE = 7
    GALE = 8
    STRONG_GALE = 9
    STORM = 10
    VIOLENT_STORM = 11
    HURRICANE = 12


class SeaState(IntEnum):
    """Douglas sea state scale"""
    CALM_GLASSY = 0
    CALM_RIPPLED = 1
    SMOOTH = 2
    SLIGHT = 3
    MODERATE = 4
    ROUGH = 5
    VERY_ROUGH = 6
    HIGH = 7
    VERY_HIGH = 8
    PHENOMENAL = 9


@dataclass
class Vector3:
    """3D vector for physics calculations"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: 'Vector3') -> 'Vector3':
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: 'Vector3') -> 'Vector3':
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> 'Vector3':
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def length(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def length_xy(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)


@dataclass
class EnvironmentParams:
    """Environment simulation parameters"""
    # Wind
    wind_speed_ms: float = 5.0
    wind_direction_deg: float = 0.0
    wind_turbulence: float = 0.1
    wind_gust_factor: float = 1.2

    # Waves
    wave_enable: bool = False
    wave_height_m: float = 0.3
    wave_period_s: float = 4.0
    wave_direction_deg: float = 0.0

    # Tide/current
    tide_speed_ms: float = 0.0
    tide_direction_deg: float = 0.0

    def get_beaufort(self) -> BeaufortScale:
        """Get Beaufort scale from wind speed"""
        return wind_speed_to_beaufort(self.wind_speed_ms)

    def get_sea_state(self) -> SeaState:
        """Get sea state from wave height"""
        return wave_height_to_sea_state(self.wave_height_m)


@dataclass
class SimState:
    """Simulation state"""
    # Position
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    altitude_m: float = 0.0

    # Velocity
    velocity_ef: Vector3 = field(default_factory=Vector3)
    velocity_bf: Vector3 = field(default_factory=Vector3)
    velocity_water: Vector3 = field(default_factory=Vector3)

    # Attitude (radians)
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0

    # Angular rates
    gyro: Vector3 = field(default_factory=Vector3)

    # Acceleration
    accel_bf: Vector3 = field(default_factory=Vector3)

    # Wingsail
    wingsail_angle_deg: float = 0.0
    flap_angle_deg: float = 0.0
    normalized_control: float = 0.0

    # Apparent wind
    apparent_wind_speed_ms: float = 0.0
    apparent_wind_dir_deg: float = 0.0

    # Performance
    vmg: float = 0.0
    heel_angle_deg: float = 0.0
    leeway_deg: float = 0.0

    # Computed properties
    @property
    def speed_ms(self) -> float:
        return self.velocity_bf.x

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.yaw_rad)

    @property
    def position_m(self) -> Tuple[float, float]:
        """Position in meters from origin"""
        return (self.lat_deg * 111320.0, self.lon_deg * 111320.0)


# Utility functions
def beaufort_to_wind_speed(beaufort: BeaufortScale) -> float:
    """Convert Beaufort scale to wind speed in m/s"""
    speeds = [0.25, 1.0, 2.45, 4.45, 6.7, 9.35, 12.3, 15.5, 18.95, 22.6, 26.45, 30.55, 35.0]
    return speeds[min(int(beaufort), 12)]


def sea_state_to_wave_height(state: SeaState) -> float:
    """Convert sea state to significant wave height in meters"""
    heights = [0.0, 0.05, 0.3, 0.875, 1.875, 3.25, 5.0, 7.5, 11.5, 14.0]
    return heights[min(int(state), 9)]


def wind_speed_to_beaufort(speed_ms: float) -> BeaufortScale:
    """Convert wind speed to Beaufort scale"""
    thresholds = [0.5, 1.6, 3.4, 5.5, 8.0, 10.8, 13.9, 17.2, 20.8, 24.5, 28.5, 32.7]
    for i, thresh in enumerate(thresholds):
        if speed_ms < thresh:
            return BeaufortScale(i)
    return BeaufortScale.HURRICANE


def wave_height_to_sea_state(height_m: float) -> SeaState:
    """Convert wave height to sea state"""
    thresholds = [0.0, 0.1, 0.5, 1.25, 2.5, 4.0, 6.0, 9.0, 14.0]
    for i, thresh in enumerate(thresholds):
        if height_m < thresh:
            return SeaState(max(0, i - 1))
    return SeaState.PHENOMENAL


class SailboatSimulator:
    """
    Pure Python sailboat physics simulator

    Implements the same physics as SIM_Sailboat_USV.cpp
    """

    # Lift/drag curves (angle index every 10 degrees, 0-180)
    LIFT_CURVE_ROTATION = [
        0.00, 0.60, 1.10, 1.20, 1.00, 0.80, 0.60, 0.45, 0.25, 0.00,
        -0.25, -0.45, -0.60, -0.80, -1.00, -1.20, -1.10, -0.60, 0.00
    ]
    DRAG_CURVE_ROTATION = [
        0.01, 0.02, 0.05, 0.15, 0.35, 0.55, 0.75, 0.90, 1.00, 1.05,
        1.00, 0.90, 0.75, 0.55, 0.35, 0.15, 0.05, 0.02, 0.01
    ]
    LIFT_CURVE_FLAP = [
        0.30, 0.80, 1.30, 1.50, 1.35, 1.10, 0.85, 0.60, 0.35, 0.10,
        -0.15, -0.40, -0.65, -0.90, -1.15, -1.40, -1.20, -0.70, -0.20
    ]
    DRAG_CURVE_FLAP = [
        0.02, 0.03, 0.06, 0.18, 0.40, 0.62, 0.82, 0.95, 1.05, 1.10,
        1.05, 0.95, 0.82, 0.62, 0.40, 0.18, 0.06, 0.03, 0.02
    ]

    def __init__(self):
        self.wingsail_type = WingsailType.WINGSAIL_FLAP
        self.env = EnvironmentParams()
        self.state = SimState()

        # Control inputs
        self.rudder_input = 0.0
        self.wingsail_input = 0.0
        self.throttle_input = 0.0

        # Physical parameters
        self.sail_area = 1.5  # m^2
        self.hull_mass = 30.0  # kg
        self.hull_length = 2.0  # m
        self.hull_beam = 1.0  # m
        self.rudder_max_angle = 35.0  # deg
        self.turning_circle = 3.0  # m
        self.rotation_angle_max = 40.0  # deg
        self.flap_angle_max = 16.0  # deg

        # Internal state
        self.wave_phase = 0.0
        self.sim_time = 0.0
        self.motor_enabled = False

        # Data recording
        self.history: List[SimState] = []
        self.record_interval = 0.1  # seconds
        self.last_record_time = 0.0

    def init(self, wingsail_type: WingsailType = WingsailType.WINGSAIL_FLAP):
        """Initialize simulator with specified wingsail type"""
        self.wingsail_type = wingsail_type
        self.reset()

    def reset(self):
        """Reset simulation state"""
        self.state = SimState()
        self.rudder_input = 0.0
        self.wingsail_input = 0.0
        self.throttle_input = 0.0
        self.wave_phase = 0.0
        self.sim_time = 0.0
        self.history.clear()
        self.last_record_time = 0.0

    def set_environment(self, env: EnvironmentParams):
        """Set environment parameters"""
        self.env = env

    def set_beaufort(self, beaufort: BeaufortScale):
        """Set wind conditions by Beaufort scale"""
        self.env.wind_speed_ms = beaufort_to_wind_speed(beaufort)
        self.env.wind_turbulence = 0.05 + int(beaufort) * 0.02
        self.env.wind_gust_factor = 1.0 + int(beaufort) * 0.05

    def set_sea_state(self, sea_state: SeaState):
        """Set wave conditions by sea state"""
        self.env.wave_enable = sea_state != SeaState.CALM_GLASSY
        self.env.wave_height_m = sea_state_to_wave_height(sea_state)
        self.env.wave_period_s = 2.0 + math.sqrt(self.env.wave_height_m) * 3.0

    def set_rudder(self, rudder: float):
        """Set rudder input (-1 to +1)"""
        self.rudder_input = max(-1.0, min(1.0, rudder))

    def set_wingsail(self, wingsail: float):
        """Set wingsail/flap input (-1 to +1)"""
        self.wingsail_input = max(-1.0, min(1.0, wingsail))
        self.state.normalized_control = self.wingsail_input

    def set_throttle(self, throttle: float):
        """Set motor throttle (0 to 1)"""
        self.throttle_input = max(0.0, min(1.0, throttle))

    def set_heading(self, heading_deg: float):
        """Set initial heading in degrees"""
        self.state.yaw_rad = math.radians(heading_deg)

    # Preset scenarios
    def set_scenario_upwind(self):
        """Configure for upwind sailing test"""
        self.env.wind_direction_deg = 0.0
        self.state.yaw_rad = math.radians(45.0)
        self.set_beaufort(BeaufortScale.MODERATE_BREEZE)
        self.set_sea_state(SeaState.SLIGHT)

    def set_scenario_downwind(self):
        """Configure for downwind sailing test"""
        self.env.wind_direction_deg = 0.0
        self.state.yaw_rad = math.radians(180.0)
        self.set_beaufort(BeaufortScale.FRESH_BREEZE)
        self.set_sea_state(SeaState.MODERATE)

    def set_scenario_beam_reach(self):
        """Configure for beam reach test"""
        self.env.wind_direction_deg = 0.0
        self.state.yaw_rad = math.radians(90.0)
        self.set_beaufort(BeaufortScale.GENTLE_BREEZE)
        self.set_sea_state(SeaState.SMOOTH)

    def set_scenario_storm(self):
        """Configure for storm survival test"""
        self.env.wind_direction_deg = 45.0
        self.state.yaw_rad = math.radians(270.0)
        self.set_beaufort(BeaufortScale.GALE)
        self.set_sea_state(SeaState.ROUGH)
        self.env.tide_speed_ms = 1.5
        self.env.tide_direction_deg = 90.0

    def _wrap_180(self, angle: float) -> float:
        """Wrap angle to -180 to +180"""
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def _wrap_2pi(self, angle: float) -> float:
        """Wrap angle to 0 to 2*pi"""
        while angle >= 2 * math.pi:
            angle -= 2 * math.pi
        while angle < 0:
            angle += 2 * math.pi
        return angle

    def _interpolate_curve(self, curve: list, angle_deg: float) -> float:
        """Interpolate from lift/drag curve"""
        angle_deg = abs(self._wrap_180(angle_deg))
        if angle_deg >= 180.0:
            return curve[-1]
        if angle_deg <= 0.0:
            return curve[0]

        idx = int(angle_deg / 10.0)
        if idx >= len(curve) - 1:
            return curve[-1]

        frac = (angle_deg - idx * 10.0) / 10.0
        return curve[idx] + frac * (curve[idx + 1] - curve[idx])

    def _update_apparent_wind(self):
        """Calculate apparent wind"""
        # True wind vector (direction it's blowing TO)
        wind_dir_rad = math.radians(self.env.wind_direction_deg + 180.0)
        true_wind = Vector3(
            self.env.wind_speed_ms * math.cos(wind_dir_rad),
            self.env.wind_speed_ms * math.sin(wind_dir_rad),
            0.0
        )

        # Add turbulence
        if self.env.wind_turbulence > 0:
            turb = self.env.wind_turbulence * self.env.wind_speed_ms
            t = self.sim_time
            true_wind.x += turb * math.sin(t * 2.1 + 0.5)
            true_wind.y += turb * math.cos(t * 1.7 + 1.2)

        # Apparent wind = true wind - boat velocity
        apparent_ef = true_wind - self.state.velocity_ef

        # Convert to body frame
        cos_yaw = math.cos(self.state.yaw_rad)
        sin_yaw = math.sin(self.state.yaw_rad)
        apparent_bf = Vector3(
            apparent_ef.x * cos_yaw + apparent_ef.y * sin_yaw,
            -apparent_ef.x * sin_yaw + apparent_ef.y * cos_yaw,
            0.0
        )

        self.state.apparent_wind_speed_ms = apparent_bf.length_xy()
        self.state.apparent_wind_dir_deg = math.degrees(math.atan2(apparent_bf.y, apparent_bf.x))

    def _calc_wingsail_forces(self) -> Tuple[float, float]:
        """Calculate lift and drag from wingsail"""
        app_speed = self.state.apparent_wind_speed_ms
        app_angle = self.state.apparent_wind_dir_deg

        if self.wingsail_type == WingsailType.WINGSAIL_ROTATION:
            wingsail_angle = self.wingsail_input * self.rotation_angle_max
            self.state.wingsail_angle_deg = wingsail_angle
            aoa = app_angle - wingsail_angle
            cl = self._interpolate_curve(self.LIFT_CURVE_ROTATION, aoa)
            cd = self._interpolate_curve(self.DRAG_CURVE_ROTATION, aoa)

        elif self.wingsail_type == WingsailType.WINGSAIL_FLAP:
            flap_angle = self.wingsail_input * self.flap_angle_max
            self.state.flap_angle_deg = flap_angle
            # Self-aligning wingsail
            wingsail_angle = app_angle - (5.0 if app_angle > 0 else -5.0)
            self.state.wingsail_angle_deg = wingsail_angle
            aoa = app_angle - wingsail_angle
            flap_cl_boost = flap_angle / self.flap_angle_max * 0.5
            flap_cd_boost = abs(flap_angle) / self.flap_angle_max * 0.3
            cl = self._interpolate_curve(self.LIFT_CURVE_FLAP, aoa) + flap_cl_boost
            cd = self._interpolate_curve(self.DRAG_CURVE_FLAP, aoa) + flap_cd_boost
            if flap_angle < 0:
                cl = -cl

        else:  # WINGSAIL_FREE
            self.state.wingsail_angle_deg = app_angle
            self.state.flap_angle_deg = 0.0
            cl = 0.1 * (-1 if app_angle > 0 else 1)
            cd = 0.02

        # Dynamic pressure
        q = 0.5 * 1.225 * app_speed * app_speed
        lift = q * self.sail_area * cl
        drag = q * self.sail_area * cd

        return lift, drag

    def _get_yaw_rate(self, rudder: float, speed: float) -> float:
        """Calculate yaw rate from rudder and speed"""
        if abs(rudder) < 0.01 or abs(speed) < 0.1:
            return 0.0

        rudder_angle = rudder * self.rudder_max_angle
        turn_radius = self.turning_circle / (2.0 * math.sin(abs(math.radians(rudder_angle))))
        turn_radius = max(0.5, turn_radius)

        yaw_rate = speed / turn_radius
        if rudder < 0:
            yaw_rate = -yaw_rate

        return yaw_rate

    def update(self, dt: float):
        """Update simulation by dt seconds"""
        self.sim_time += dt

        # Update apparent wind
        self._update_apparent_wind()

        # Tide effects
        if self.env.tide_speed_ms > 0.01:
            tide_dir_rad = math.radians(self.env.tide_direction_deg)
            tide_vel = Vector3(
                self.env.tide_speed_ms * math.cos(tide_dir_rad),
                self.env.tide_speed_ms * math.sin(tide_dir_rad),
                0.0
            )
            self.state.velocity_water = self.state.velocity_ef - tide_vel
        else:
            self.state.velocity_water = self.state.velocity_ef

        # Wingsail forces
        sail_lift, sail_drag = self._calc_wingsail_forces()

        # Convert to body frame
        app_wind_rad = math.radians(self.state.apparent_wind_dir_deg)
        sail_force_x = -sail_drag * math.cos(app_wind_rad) + sail_lift * math.sin(app_wind_rad)
        sail_force_y = -sail_drag * math.sin(app_wind_rad) - sail_lift * math.cos(app_wind_rad)

        # Hull drag
        speed = self.state.velocity_water.length_xy()
        q_water = 0.5 * 1025.0 * speed * speed
        hull_drag = q_water * 1.5 * 0.05  # wetted_area * Cd

        # Motor force
        motor_force = 0.0
        if self.motor_enabled and self.throttle_input > 0:
            max_motor_force = self.hull_mass * 0.5
            speed_factor = 1.0 - min(1.0, self.state.velocity_bf.x / 3.0)
            motor_force = self.throttle_input * max_motor_force * speed_factor

        # Total forces
        total_force_x = sail_force_x + motor_force - hull_drag
        total_force_y = sail_force_y

        # Accelerations
        mass_x = self.hull_mass * 1.05  # added mass
        mass_y = self.hull_mass * 1.8
        self.state.accel_bf = Vector3(
            total_force_x / mass_x,
            total_force_y / mass_y,
            0.0
        )

        # Yaw dynamics
        yaw_rate = self._get_yaw_rate(self.rudder_input, self.state.velocity_bf.x)
        self.state.gyro.z = self.state.gyro.z * 0.9 + yaw_rate * 0.1

        # Heel from sail moment
        sail_height = math.sqrt(self.sail_area * 4.0)
        heel_moment = sail_lift * sail_height * 0.5
        heel_restoring = -self.hull_mass * 9.81 * math.sin(self.state.roll_rad) * self.hull_beam * 0.3
        heel_inertia = self.hull_mass * self.hull_beam**2 / 12.0
        self.state.gyro.x += ((heel_moment + heel_restoring) / heel_inertia) * dt * 0.1

        # Wave effects
        if self.env.wave_enable and self.env.wave_height_m > 0.01:
            wave_speed = math.sqrt(9.81 * self.env.wave_period_s / (2 * math.pi))
            wavelength = wave_speed * self.env.wave_period_s
            wave_dir_rad = math.radians(self.env.wave_direction_deg)
            boat_wave_speed = (self.state.velocity_ef.x * math.cos(wave_dir_rad) +
                               self.state.velocity_ef.y * math.sin(wave_dir_rad))
            relative_speed = wave_speed - boat_wave_speed
            self.wave_phase += (relative_speed / wavelength) * 2 * math.pi * dt
            self.wave_phase = self.wave_phase % (2 * math.pi)

            wave_slope = (self.env.wave_height_m / 2.0) * (2 * math.pi / wavelength) * math.cos(self.wave_phase)
            wave_angle = math.atan(wave_slope)
            heading_diff = wave_dir_rad - self.state.yaw_rad
            wave_roll = math.sin(heading_diff) * wave_angle
            wave_pitch = math.cos(heading_diff) * wave_angle

            wave_gain = 2.0
            self.state.gyro.x += (wave_roll - self.state.roll_rad) * wave_gain * dt
            self.state.gyro.y += (wave_pitch - self.state.pitch_rad) * wave_gain * dt

        # Integrate angular rates
        self.state.roll_rad += self.state.gyro.x * dt
        self.state.pitch_rad += self.state.gyro.y * dt
        self.state.yaw_rad += self.state.gyro.z * dt

        # Damp angles
        self.state.roll_rad *= 0.98
        self.state.pitch_rad *= 0.98
        self.state.gyro.x *= 0.95
        self.state.gyro.y *= 0.95
        self.state.gyro.z *= 0.98

        # Wrap yaw
        self.state.yaw_rad = self._wrap_2pi(self.state.yaw_rad + math.pi) - math.pi

        # Update velocity
        self.state.velocity_bf.x += self.state.accel_bf.x * dt
        self.state.velocity_bf.y += self.state.accel_bf.y * dt
        self.state.velocity_bf.x *= 0.995
        self.state.velocity_bf.y *= 0.95

        # Convert to earth frame
        cos_yaw = math.cos(self.state.yaw_rad)
        sin_yaw = math.sin(self.state.yaw_rad)
        self.state.velocity_ef = Vector3(
            self.state.velocity_bf.x * cos_yaw - self.state.velocity_bf.y * sin_yaw,
            self.state.velocity_bf.x * sin_yaw + self.state.velocity_bf.y * cos_yaw,
            0.0
        )

        # Add tide
        if self.env.tide_speed_ms > 0.01:
            tide_dir_rad = math.radians(self.env.tide_direction_deg)
            self.state.velocity_ef.x += self.env.tide_speed_ms * math.cos(tide_dir_rad)
            self.state.velocity_ef.y += self.env.tide_speed_ms * math.sin(tide_dir_rad)

        # Update position
        self.state.lat_deg += self.state.velocity_ef.x * dt / 111320.0
        self.state.lon_deg += self.state.velocity_ef.y * dt / (111320.0 * math.cos(math.radians(self.state.lat_deg)))

        # Calculate VMG
        target_dir_rad = math.radians(self.env.wind_direction_deg)
        boat_speed = self.state.velocity_ef.length_xy()
        boat_dir_rad = math.atan2(self.state.velocity_ef.y, self.state.velocity_ef.x)
        self.state.vmg = boat_speed * math.cos(boat_dir_rad - target_dir_rad)

        # Update heel angle
        self.state.heel_angle_deg = math.degrees(self.state.roll_rad)

        # Record history
        if self.sim_time - self.last_record_time >= self.record_interval:
            self.history.append(SimState(
                lat_deg=self.state.lat_deg,
                lon_deg=self.state.lon_deg,
                velocity_bf=Vector3(self.state.velocity_bf.x, self.state.velocity_bf.y, 0),
                velocity_ef=Vector3(self.state.velocity_ef.x, self.state.velocity_ef.y, 0),
                yaw_rad=self.state.yaw_rad,
                roll_rad=self.state.roll_rad,
                vmg=self.state.vmg,
                heel_angle_deg=self.state.heel_angle_deg,
                apparent_wind_speed_ms=self.state.apparent_wind_speed_ms,
                apparent_wind_dir_deg=self.state.apparent_wind_dir_deg,
                wingsail_angle_deg=self.state.wingsail_angle_deg,
                flap_angle_deg=self.state.flap_angle_deg,
            ))
            self.last_record_time = self.sim_time

    def run(self, duration: float, dt: float = 0.02) -> List[SimState]:
        """Run simulation for specified duration"""
        steps = int(duration / dt)
        for _ in range(steps):
            self.update(dt)
        return self.history

    def get_state(self) -> SimState:
        """Get current simulation state"""
        return self.state

    def get_history(self) -> List[SimState]:
        """Get recorded state history"""
        return self.history
