# -*- coding: utf-8 -*-
# ----------------------------------------
# Stepper motor and actuator control utils
# ----------------------------------------# -*- coding: utf-8 -*-
# ---------------------------------
# Stepper motor S-curve motion with PID control and encoder logging
# ---------------------------------
import time
import time
import math
import sys
import os
import re
import csv
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR and SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

try:
    import lgpio
except Exception:
    lgpio = None

try:
    import smbus
except Exception:
    smbus = None

try:
    from graph import plot_results as plot_csv_results
except Exception:
    plot_csv_results = None
# -------------------- Pins --------------------
DIR_PIN_NAMA_17  = 24
STEP_PIN_NAMA_17 = 23
ENA_PIN_NAMA_17  = 25

DIR_PIN_NAMA_23  = 20
STEP_PIN_NAMA_23 = 21
ENA_PIN_NAMA_23  = 16

IN1_PIN = 5
IN2_PIN = 6
PWM_PIN = 13

ENC_PIN_A = 2  # I2C SDA (for AS5600 magnetic encoder)
ENC_PIN_B = 3  # I2C SCL (for AS5600 magnetic encoder)

# -------------------- Motion params --------------------
MIN_DELAY_17 = 0.0005
MAX_DELAY_17 = 0.002
STEPS_17     = 1320

MIN_DELAY_23 = 0.0003   # fastest (shortest delay)
MAX_DELAY_23 = 0.0015   # slowest (longest delay)
STEPS_23     = 10000

# Safety
MIN_SAFE_DELAY = 0.00025  # 250 us
TX_BACKLOG = 8

# Actuator / encoder
ACT_TIME       = 7
ENC_SAMPLE_DT  = 0.015
MUX_ADDR       = 0x70
ENCODER_ADDR   = 0x36  # AS5600 I2C address
ENC_REG_ANGLE  = 0x0E  # Angle register for multiplexed encoders

DEG_PER_STEP = 0.018  # 10000 steps = 180 deg
PULSES_PER_REV = 360.0 / DEG_PER_STEP if DEG_PER_STEP else float("nan")
PULSE_SPEED_FILTER_TAU = 0.05  # seconds, low-pass filter time constant

# -------------------- Output folders --------------------
LOG_DIR = os.path.join(SCRIPT_DIR, "logs")
os.makedirs(LOG_DIR, exist_ok=True)

def _safe_name(s: str) -> str:
    return re.sub(r'[^A-Za-z0-9._-]+', '_', s)

def plot_time_series(*_args, **_kwargs):
    """Graph plotting disabled."""
    return None

# -------------------- Encoder via PCA9548A --------------------
class EncoderMux:
    def __init__(self, bus_num=1, settle=0.002, retries=3, retry_wait=0.001):
        if smbus is None:
            raise RuntimeError("smbus not available. Run on Raspberry Pi with I2C enabled.")
        self.bus = smbus.SMBus(bus_num)
        self.settle = settle
        self.retries = retries
        self.retry_wait = retry_wait

    def select_channel(self, ch: int):
        self.bus.write_byte(MUX_ADDR, 1 << ch)
        time.sleep(self.settle)

    def read_angle_deg_once(self) -> float:
        data = self.bus.read_i2c_block_data(ENCODER_ADDR, ENC_REG_ANGLE, 2)
        raw  = ((data[0] << 8) | data[1]) & 0x0FFF
        return raw * 360.0 / 4096.0

    def read_angle_deg(self) -> float:
        for _ in range(self.retries):
            try:
                return self.read_angle_deg_once()
            except Exception:
                time.sleep(self.retry_wait)
        raise

    def try_read_channel(self, ch: int):
        try:
            self.select_channel(ch)
            return self.read_angle_deg()
        except Exception:
            return float("nan")

    def detect_working_channels(self, candidates=None):
        if candidates is None:
            candidates = list(range(8))
        ok = []
        for ch in candidates:
            v = self.try_read_channel(ch)
            if not (np.isnan(v) or np.isinf(v)):
                ok.append(ch)
        try:
            self.bus.write_byte(MUX_ADDR, 0x00)
        except Exception:
            pass
        return ok

# -------------------- AS5600 Magnetic Encoder (I2C) --------------------
class AS5600Encoder:
    """AS5600 absolute magnetic encoder via I2C on GPIO 2,3 (SDA,SCL)"""
    def __init__(self, bus_num=1, i2c_addr=0x36, pulses_per_rev=None):
        """
        Initialize AS5600 encoder.
        
        Args:
            bus_num: I2C bus number (usually 1 for Raspberry Pi)
            i2c_addr: I2C address of AS5600 (default 0x36)
            pulses_per_rev: Pulses per revolution for compatibility
        """
        if smbus is None:
            raise RuntimeError("smbus not available. Install: sudo apt-get install python3-smbus")
        
        self.bus = smbus.SMBus(bus_num)
        self.addr = i2c_addr
        self.pulses_per_rev = pulses_per_rev if pulses_per_rev else PULSES_PER_REV
        self.angle_reg = 0x0C  # Raw angle register (12-bit)
        self.last_angle_deg = 0.0
        self.offset_angle = 0.0  # For reset functionality
        
        # Test read to verify connection
        try:
            self.read_angle_deg()
        except Exception as e:
            raise RuntimeError(f"AS5600 not responding at 0x{i2c_addr:02X}: {e}")
    
    def read_angle_deg(self):
        """Read absolute angle from AS5600 (0-360 degrees)"""
        try:
            # Read 2 bytes from raw angle register
            data = self.bus.read_i2c_block_data(self.addr, self.angle_reg, 2)
            # Combine into 12-bit value
            raw = ((data[0] << 8) | data[1]) & 0x0FFF
            # Convert to degrees (0-360)
            angle = raw * 360.0 / 4096.0
            # Apply offset for reset functionality
            angle = (angle - self.offset_angle) % 360.0
            self.last_angle_deg = angle
            return angle
        except Exception as e:
            # Return last known value on error
            return self.last_angle_deg
    
    def read_pulses(self):
        """Convert current angle to pulse count"""
        angle = self.read_angle_deg()
        if self.pulses_per_rev > 0:
            return int((angle / 360.0) * self.pulses_per_rev)
        return 0
    
    def update(self):
        """Update encoder reading (called periodically)"""
        self.read_angle_deg()
    
    def try_read_channel(self, ch=None):
        """Compatibility method for encoder interface"""
        return self.read_angle_deg()
    
    def select_channel(self, ch=None):
        """Compatibility method (no-op for single encoder)"""
        pass
    
    def reset(self):
        """Reset: set current position as zero reference"""
        self.offset_angle = self.read_angle_deg_raw()
        self.last_angle_deg = 0.0
    
    def read_angle_deg_raw(self):
        """Read raw angle without offset"""
        try:
            data = self.bus.read_i2c_block_data(self.addr, self.angle_reg, 2)
            raw = ((data[0] << 8) | data[1]) & 0x0FFF
            return raw * 360.0 / 4096.0
        except Exception:
            return 0.0
    
    def get_magnet_status(self):
        """Check magnet detection status
        Returns: (detected, too_weak, too_strong)
        """
        try:
            status = self.bus.read_byte_data(self.addr, 0x0B)
            md = (status >> 5) & 1  # Magnet detected
            ml = (status >> 4) & 1  # Magnet too weak
            mh = (status >> 3) & 1  # Magnet too strong
            return (bool(md), bool(ml), bool(mh))
        except Exception:
            return (False, False, False)
    
    def get_agc_value(self):
        """Get Automatic Gain Control value (0-255)
        Optimal range: 128 ± 32
        """
        try:
            return self.bus.read_byte_data(self.addr, 0x1A)
        except Exception:
            return 0

# -------------------- GPIO Quadrature Encoder (Legacy) --------------------
class GPIOEncoder:
    """Quadrature encoder reader using GPIO pins 2 and 3.
    
    NOTE: This class is for actual quadrature encoders with A/B pulse signals.
    If using AS5600 magnetic encoder, use AS5600Encoder instead.
    """
    def __init__(self, h, pin_a=ENC_PIN_A, pin_b=ENC_PIN_B, pulses_per_rev=None):
        """
        Initialize GPIO encoder.
        
        Args:
            h: GPIO chip handle
            pin_a: GPIO pin for encoder A channel
            pin_b: GPIO pin for encoder B channel
            pulses_per_rev: Pulses per revolution (if None, uses PULSES_PER_REV)
        """
        if lgpio is None:
            raise RuntimeError("lgpio not available. Run on Raspberry Pi.")
        self.h = h
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.pulses_per_rev = pulses_per_rev if pulses_per_rev is not None else PULSES_PER_REV
        
        # Claim pins as inputs
        lgpio.gpio_claim_input(h, pin_a)
        lgpio.gpio_claim_input(h, pin_b)
        
        # Initialize encoder state
        self.last_state_a = lgpio.gpio_read(h, pin_a)
        self.last_state_b = lgpio.gpio_read(h, pin_b)
        self.pulse_count = 0
        self.last_angle_deg = 0.0
        
        # Quadrature state machine: 00, 01, 11, 10
        # State transitions determine direction
        self.last_quad_state = (self.last_state_a << 1) | self.last_state_b
        
    def read_pulses(self):
        """Read current pulse count."""
        return self.pulse_count
    
    def read_angle_deg(self):
        """Read current angle in degrees."""
        if self.pulses_per_rev > 0:
            angle = (self.pulse_count / self.pulses_per_rev) * 360.0
            self.last_angle_deg = angle
            return angle
        return self.last_angle_deg
    
    def update(self):
        """Update encoder state by reading GPIO pins and counting pulses."""
        try:
            state_a = lgpio.gpio_read(self.h, self.pin_a)
            state_b = lgpio.gpio_read(self.h, self.pin_b)
            
            # Current quadrature state (2-bit: A, B)
            # Encoding: state = (A << 1) | B
            # 00 -> 0, 01 -> 1, 10 -> 2, 11 -> 3
            quad_state = (state_a << 1) | state_b
            
            # Detect state change
            if quad_state != self.last_quad_state:
                # Quadrature decoding: determine direction from state transition
                # Valid forward transitions: 0->1->3->2->0
                # Valid reverse transitions: 0->2->3->1->0
                
                # Calculate direction based on state transition
                # Forward: increment, Reverse: decrement
                # Use modulo arithmetic to handle wrap-around
                diff = (quad_state - self.last_quad_state) % 4
                
                if diff == 1:
                    # Forward transition (0->1, 1->3, 3->2, 2->0)
                    self.pulse_count += 1
                elif diff == 3:
                    # Reverse transition (0->2->3->1->0, which is -1 mod 4)
                    self.pulse_count -= 1
                # If diff == 2, it's an invalid transition (skip), ignore it
                
                self.last_quad_state = quad_state
                self.last_state_a = state_a
                self.last_state_b = state_b
                
        except Exception:
            pass  # Ignore read errors
    
    def reset(self):
        """Reset pulse count to zero."""
        self.pulse_count = 0
        self.last_angle_deg = 0.0
    
    def try_read_channel(self, ch=None):
        """Compatibility method: read angle in degrees."""
        self.update()
        return self.read_angle_deg()
    
    def select_channel(self, ch=None):
        """Compatibility method: no-op for GPIO encoder."""
        pass

# -------------------- Stepper helpers --------------------
def enable_motor(h, ena_pin, enable=True):
    state = 0 if enable else 1
    lgpio.gpio_write(h, ena_pin, state)

def stop_actuator(h):
    lgpio.gpio_write(h, IN1_PIN, 0)
    lgpio.gpio_write(h, IN2_PIN, 0)
    lgpio.tx_pwm(h, PWM_PIN, 1000, 0)

def extend(h, speed=1.0):
    duty = int(max(0, min(100, speed * 100)))
    lgpio.gpio_write(h, IN1_PIN, 1)
    lgpio.gpio_write(h, IN2_PIN, 0)
    lgpio.tx_pwm(h, PWM_PIN, 1000, duty)

def retract(h, speed=1.0):
    duty = int(max(0, min(100, speed * 100)))
    lgpio.gpio_write(h, IN1_PIN, 0)
    lgpio.gpio_write(h, IN2_PIN, 1)
    lgpio.tx_pwm(h, PWM_PIN, 1000, duty)

def smooth_cos_delay(i, n, min_delay, max_delay, gamma=1.0):
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)
    if n <= 1:
        return min_delay
    t_norm = i / (n - 1)
    if gamma not in (None, 1.0):
        if gamma <= 0:
            gamma = 1.0
        t_norm = min(max(t_norm, 0.0), 1.0) ** gamma
    k = 0.5 * (1 - math.cos(math.pi * t_norm))
    return max_delay - (max_delay - min_delay) * k

# Profile builder to support symmetric S-curve and asymmetric S-curve
def build_delay_profile(
    total_steps: int,
    min_delay: float,
    max_delay: float,
    accel_ratio,
    decel_ratio=None,
    gamma_accel: float = 1.0,
    gamma_decel=None,
):
    """Create a list of inter-pulse delays that follow an S-curve or AS-curve.

    - If decel_ratio is None, uses accel_ratio (symmetric duration).
    - If gamma_decel is None and decel_ratio equals accel_ratio, decel is exact
      reverse of accel (perfect mirror symmetry).
    - Otherwise decel is generated independently using its gamma.
    """
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)

    # Support tuple inputs for ratios
    if isinstance(accel_ratio, (tuple, list)) and len(accel_ratio) >= 2 and decel_ratio is None:
        accel_ratio, decel_ratio = float(accel_ratio[0]), float(accel_ratio[1])
    if decel_ratio is None:
        decel_ratio = accel_ratio

    accel_steps = int(max(0, round(total_steps * float(accel_ratio))))
    decel_steps = int(max(0, round(total_steps * float(decel_ratio))))

    # Ensure we do not exceed total steps; scale down proportionally if needed
    total_ad = accel_steps + decel_steps
    if total_ad > total_steps:
        scale = total_steps / float(total_ad)
        accel_steps = int(round(accel_steps * scale))
        decel_steps = max(0, total_steps - accel_steps)
    const_steps = max(0, total_steps - accel_steps - decel_steps)

    # Accel delays
    accel_delays = [
        smooth_cos_delay(i, max(accel_steps, 1), min_delay, max_delay, gamma=gamma_accel)
        for i in range(accel_steps)
    ]

    # Decel delays
    if gamma_decel is None and decel_steps == accel_steps:
        decel_delays = list(reversed(accel_delays))
    else:
        if gamma_decel is None:
            gamma_decel = gamma_accel
        decel_delays = [
            smooth_cos_delay(decel_steps - 1 - i, max(decel_steps, 1), min_delay, max_delay, gamma=gamma_decel)
            for i in range(decel_steps)
        ]

    delays = []
    delays.extend(accel_delays)
    if const_steps > 0:
        delays.extend([min_delay] * const_steps)
    delays.extend(decel_delays)
    return delays, accel_steps, const_steps, decel_steps

# -------------------- Math helpers --------------------
def unwrap_deg(deg_series: np.ndarray) -> np.ndarray:
    return np.degrees(np.unwrap(np.radians(deg_series)))

def finite_diff(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    if len(y) < 2 or len(y) != len(t):
        return np.zeros_like(y)
    dy = np.empty_like(y)
    dy[0]  = (y[1] - y[0]) / (t[1] - t[0] + 1e-12)
    dy[-1] = (y[-1] - y[-2]) / (t[-1] - t[-2] + 1e-12)
    for i in range(1, len(y)-1):
        dt = (t[i+1] - t[i-1])
        dy[i] = (y[i+1] - y[i-1]) / (dt + 1e-12)
    return dy

def cumtrapz(y: np.ndarray, x: np.ndarray):
    if y is None or x is None or len(y) < 2 or len(y) != len(x):
        return np.zeros_like(x)
    out = np.zeros_like(y)
    for i in range(1, len(y)):
        dx = x[i] - x[i-1]
        out[i] = out[i-1] + 0.5 * (y[i] + y[i-1]) * dx
    return out

# -------------------- PID Performance Metrics --------------------
def calculate_pid_metrics(t, target, measured, settling_tolerance=0.02):
    """Calculate PID performance metrics: RMS error, overshoot, settling time, steady-state error.
    
    Args:
        t: time array
        target: target value array
        measured: measured value array
        settling_tolerance: tolerance for settling time (default 2%)
    
    Returns:
        dict with metrics or None if insufficient data
    """
    if t is None or target is None or measured is None:
        return None
    if len(t) < 10 or len(target) != len(t) or len(measured) != len(t):
        return None
    
    # Filter out NaN values
    valid_mask = np.isfinite(target) & np.isfinite(measured)
    if not valid_mask.any():
        return None
    
    t_valid = t[valid_mask]
    target_valid = target[valid_mask]
    measured_valid = measured[valid_mask]
    
    if len(t_valid) < 10:
        return None
    
    # Error signal
    error = target_valid - measured_valid
    
    # RMS Error
    rms_error = float(np.sqrt(np.mean(error ** 2)))
    
    # Max Error
    max_error = float(np.max(np.abs(error)))
    
    # Mean Absolute Error
    mae = float(np.mean(np.abs(error)))
    
    # Steady-state error (last 10% of data)
    ss_idx = int(len(error) * 0.9)
    if ss_idx < len(error) - 1:
        ss_error = float(np.mean(error[ss_idx:]))
        ss_error_abs = float(np.mean(np.abs(error[ss_idx:])))
    else:
        ss_error = float(error[-1])
        ss_error_abs = float(np.abs(error[-1]))
    
    # Overshoot calculation
    # Find the final target value (last 10% average)
    final_target = float(np.mean(target_valid[ss_idx:]))
    
    # Calculate overshoot as percentage beyond final target
    if final_target != 0:
        overshoot_val = float(np.max(measured_valid) - final_target)
        overshoot_pct = (overshoot_val / abs(final_target)) * 100.0 if final_target != 0 else 0.0
    else:
        overshoot_pct = 0.0
        overshoot_val = 0.0
    
    # Settling time (time to stay within tolerance)
    tolerance_band = abs(final_target) * settling_tolerance
    settled_mask = np.abs(measured_valid - final_target) <= tolerance_band
    
    settling_time = None
    # Find first time when it enters and stays in tolerance band
    for i in range(len(settled_mask) - 10):
        if np.all(settled_mask[i:]):
            settling_time = float(t_valid[i])
            break
    
    # Rise time (10% to 90% of final value)
    initial_val = float(measured_valid[0])
    rise_range = final_target - initial_val
    if abs(rise_range) > 1e-6:
        val_10 = initial_val + 0.1 * rise_range
        val_90 = initial_val + 0.9 * rise_range
        
        idx_10 = np.where(measured_valid >= val_10)[0]
        idx_90 = np.where(measured_valid >= val_90)[0]
        
        if len(idx_10) > 0 and len(idx_90) > 0:
            rise_time = float(t_valid[idx_90[0]] - t_valid[idx_10[0]])
        else:
            rise_time = None
    else:
        rise_time = None
    
    metrics = {
        "rms_error": rms_error,
        "max_error": max_error,
        "mae": mae,
        "steady_state_error": ss_error,
        "steady_state_error_abs": ss_error_abs,
        "overshoot_value": overshoot_val,
        "overshoot_percent": overshoot_pct,
        "settling_time": settling_time,
        "rise_time": rise_time,
        "final_target": final_target,
        "final_measured": float(measured_valid[-1])
    }
    
    return metrics

# -------------------- TX helpers (no tx_pwm for steps) --------------------
def queue_pulse(h, gpio, delay_s):
    high_us = int(delay_s * 1_000_000)
    while lgpio.tx_room(h, gpio, lgpio.TX_PWM) <= 0:
        time.sleep(delay_s * 0.25)
    lgpio.tx_pulse(h, gpio, high_us, high_us, 0, 1)

# -------------------- Rate/Delay utils --------------------
def rate_to_delay(rate_steps_per_s, min_delay, max_delay):
    if rate_steps_per_s <= 0:
        return max_delay
    d = 1.0 / (2.0 * rate_steps_per_s)
    d = max(min(d, max_delay), max(min_delay, MIN_SAFE_DELAY))
    return d

def delay_to_rate(delay_s):
    return 1.0 / (2.0 * max(delay_s, 1e-6))

def pulses_to_speed(pulse_count: int, dt: float, pulses_per_rev: float):
    if dt <= 0.0 or pulses_per_rev <= 0.0:
        return float("nan"), float("nan")
    revs = pulse_count / pulses_per_rev
    revs_per_s = revs / dt
    rpm = revs_per_s * 60.0
    rad_s = revs_per_s * math.tau
    return rpm, rad_s

def lowpass_first_order(prev_value: float, new_value: float, dt: float, tau: float) -> float:
    if not math.isfinite(new_value):
        return prev_value
    if tau <= 0.0:
        return new_value
    if not math.isfinite(prev_value):
        return new_value
    dt = max(dt, 1e-9)
    alpha = dt / (tau + dt)
    alpha = min(max(alpha, 0.0), 1.0)
    return prev_value + alpha * (new_value - prev_value)

def apply_speed_target_to_delays(
    min_delay: float,
    max_delay: float,
    target_deg_per_s: float | None
) -> tuple[float, float, float | None]:
    if target_deg_per_s is None or target_deg_per_s <= 0:
        return min_delay, max_delay, None

    fast_limit = max(min_delay, MIN_SAFE_DELAY)
    slow_limit = max(max_delay, fast_limit)

    rate_steps_per_s = target_deg_per_s / max(DEG_PER_STEP, 1e-9)
    if rate_steps_per_s <= 0:
        return min_delay, max_delay, None

    target_delay = 1.0 / (2.0 * rate_steps_per_s)
    target_delay = max(target_delay, fast_limit)

    if target_delay > slow_limit:
        slow_limit = target_delay

    min_delay = target_delay
    max_delay = slow_limit

    achieved_deg_per_s = (1.0 / (2.0 * min_delay)) * DEG_PER_STEP
    return min_delay, max_delay, achieved_deg_per_s

def export_motion_csv(
    t: np.ndarray,
    cmd_angle: np.ndarray,
    pulse_deg: np.ndarray,
    cmd_ang_vel: np.ndarray,
    pulse_vel: np.ndarray,
    filename_base: str = "motion_run",
    pid_error: np.ndarray = None,
    pid_trim: np.ndarray = None,
    target_angle: np.ndarray = None,
    measured_angle: np.ndarray = None
) -> str | None:
    if t is None or len(t) < 2:
        return None
    t = np.asarray(t, dtype=float)
    if not np.all(np.isfinite(t)):
        return None
    sample_dt = 0.005  # 5 ms
    t_start = float(t[0])
    t_end = float(t[-1])
    if t_end <= t_start:
        return None
    samples = np.arange(t_start, t_end + sample_dt * 0.5, sample_dt, dtype=float)

    def _prepare(series):
        if series is None:
            return None
        arr = np.asarray(series, dtype=float)
        if len(arr) != len(t):
            return None
        return arr

    cmd_angle_arr = _prepare(cmd_angle)
    pulse_deg_arr = _prepare(pulse_deg)
    cmd_vel_arr = _prepare(cmd_ang_vel)
    pulse_vel_arr = _prepare(pulse_vel)
    
    # PID data (optional)
    pid_error_arr = _prepare(pid_error)
    pid_trim_arr = _prepare(pid_trim)
    target_angle_arr = _prepare(target_angle)
    measured_angle_arr = _prepare(measured_angle)

    if cmd_angle_arr is None and cmd_vel_arr is None:
        return None

    def _grad(values, spacing):
        if values is None:
            return np.zeros_like(samples)
        if len(values) < 2:
            return np.zeros_like(values)
        edge_order = 2 if len(values) > 2 else 1
        return np.gradient(values, spacing, edge_order=edge_order)

    if cmd_angle_arr is None and cmd_vel_arr is not None:
        cmd_angle_arr = np.zeros_like(t, dtype=float)
        for i in range(1, len(t)):
            dt = max(t[i] - t[i - 1], 1e-6)
            cmd_angle_arr[i] = cmd_angle_arr[i - 1] + 0.5 * (cmd_vel_arr[i] + cmd_vel_arr[i - 1]) * dt

    if cmd_vel_arr is None and cmd_angle_arr is not None:
        cmd_vel_arr = _grad(cmd_angle_arr, t)

    if cmd_angle_arr is None:
        com_incremental = np.zeros_like(samples)
    else:
        com_incremental = np.interp(samples, t, cmd_angle_arr - cmd_angle_arr[0])
    com_pos = np.cumsum(np.abs(np.diff(com_incremental, prepend=com_incremental[0])))
    com_vel = np.interp(samples, t, cmd_vel_arr) if cmd_vel_arr is not None else _grad(com_incremental, sample_dt)

    if pulse_deg_arr is None:
        pul_incremental = np.zeros_like(samples)
    else:
        pul_incremental = np.interp(samples, t, pulse_deg_arr - pulse_deg_arr[0])
    pul_pos = np.cumsum(np.abs(np.diff(pul_incremental, prepend=pul_incremental[0])))

    if pulse_vel_arr is not None:
        pul_vel = np.interp(samples, t, pulse_vel_arr)
    elif len(samples) > 1:
        pul_vel = _grad(pul_incremental, sample_dt)
    else:
        pul_vel = np.zeros_like(samples)

    time_ms = (samples - t_start) * 1000.0
    
    # Interpolate PID data if available
    pid_err_interp = np.interp(samples, t, pid_error_arr) if pid_error_arr is not None else None
    pid_trim_interp = np.interp(samples, t, pid_trim_arr) if pid_trim_arr is not None else None
    target_ang_interp = np.interp(samples, t, target_angle_arr) if target_angle_arr is not None else None
    measured_ang_interp = np.interp(samples, t, measured_angle_arr) if measured_angle_arr is not None else None

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    fname = _safe_name(f"{filename_base}_{timestamp}.csv")
    path = os.path.join(LOG_DIR, fname)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        
        # Build header
        header = ["Time_ms", "com_Pos_deg", "pul_Pos_deg", "com_Vel_deg_per_s", "pul_Vel_deg_per_s"]
        if target_ang_interp is not None:
            header.append("Target_Angle_deg")
        if measured_ang_interp is not None:
            header.append("Measured_Angle_deg")
        if pid_err_interp is not None:
            header.append("PID_Error_deg")
        if pid_trim_interp is not None:
            header.append("PID_Trim_deg_per_s")
        writer.writerow(header)
        
        # Write data
        for i in range(len(samples)):
            row = [time_ms[i], com_pos[i], pul_pos[i], com_vel[i], pul_vel[i]]
            if target_ang_interp is not None:
                row.append(target_ang_interp[i])
            if measured_ang_interp is not None:
                row.append(measured_ang_interp[i])
            if pid_err_interp is not None:
                row.append(pid_err_interp[i])
            if pid_trim_interp is not None:
                row.append(pid_trim_interp[i])
            writer.writerow(row)
    return path

# -------------------- PID --------------------
class PID:
    def __init__(self, kp=2.0, ki=0.0, kd=0.0, out_min=-300.0, out_max=300.0, tau=0.02):
        # output unit: deg/s
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.tau = tau
        self._i = 0.0
        self._prev_e = 0.0
        self._prev_d = 0.0

    def reset(self):
        self._i = 0.0
        self._prev_e = 0.0
        self._prev_d = 0.0

    def update(self, e, dt, anti_windup_ref=None):
        if dt <= 0.0:
            dt = 1e-3
        p = self.kp * e
        self._i += self.ki * e * dt
        d_raw = (e - self._prev_e) / dt
        d = self._prev_d + (dt / (self.tau + dt)) * (d_raw - self._prev_d)
        out = p + self._i + self.kd * d
        if out > self.out_max:
            out = self.out_max
            if anti_windup_ref is not None and self.ki > 0:
                self._i += 0.1 * (anti_windup_ref - out)
        elif out < self.out_min:
            out = self.out_min
            if anti_windup_ref is not None and self.ki > 0:
                self._i += 0.1 * (anti_windup_ref - out)
        self._prev_e = e
        self._prev_d = d
        return out

# -------------------- Open-loop S-curve with logging --------------------
def move_stepper_scurve_with_logging(
    h, dir_pin, step_pin, total_steps, direction, min_delay, max_delay,
    accel_ratio=0.2, s_curve_gamma: float = 1.0,
    log_enc: bool = True, enc: EncoderMux = None, enc_channels=None,
    enc_sample_dt: float = ENC_SAMPLE_DT,
    decel_ratio=None,
    s_curve_gamma_decel=None,
    encoders_dict=None,
):
    if enc_channels is None:
        enc_channels = []
    if encoders_dict is None:
        encoders_dict = {}
    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)

    lgpio.gpio_write(h, dir_pin, direction)

    delays, accel_steps, const_steps, decel_steps = build_delay_profile(
        total_steps=total_steps,
        min_delay=min_delay,
        max_delay=max_delay,
        accel_ratio=accel_ratio,
        decel_ratio=decel_ratio,
        gamma_accel=s_curve_gamma,
        gamma_decel=s_curve_gamma_decel,
    )

    pulses_per_rev = PULSES_PER_REV if math.isfinite(PULSES_PER_REV) and PULSES_PER_REV > 0 else float("nan")

    t0 = time.monotonic()
    last_sample_t = t0
    last_sample_pulse_count = 0
    total_pulses = 0
    time_log = []
    enc_logs = {ch: [] for ch in enc_channels}
    cmd_rate_log = []
    pulse_rpm_log = []
    pulse_deg_log = []
    pulse_deg_per_s_log = []
    pulse_deg_per_s_filtered_log = []
    pulse_rpm_filtered_log = []
    pulse_deg_per_s_lpf = float("nan")
    current_cmd_rate = 0.0

    def sample_all(ts):
        nonlocal last_sample_t, last_sample_pulse_count, pulse_deg_per_s_lpf
        if ts - last_sample_t >= enc_sample_dt:
            if log_enc:
                for ch in enc_channels:
                    # Use encoder from dictionary if available, otherwise fall back to enc parameter
                    encoder = encoders_dict.get(ch, enc)
                    if encoder is not None:
                        # For GPIO encoder, update state before reading
                        if isinstance(encoder, GPIOEncoder):
                            encoder.update()
                        ang = encoder.try_read_channel(ch)
                        enc_logs[ch].append(ang)
            dt = ts - last_sample_t
            if dt <= 0.0:
                dt = 1e-6
            pulse_delta = total_pulses - last_sample_pulse_count
            rpm, _ = pulses_to_speed(pulse_delta, dt, pulses_per_rev)
            deg_per_s = (pulse_delta * DEG_PER_STEP) / dt
            if (not math.isfinite(rpm)) and math.isfinite(deg_per_s):
                rpm = deg_per_s / 6.0
            pulse_rpm_log.append(rpm)
            pulse_deg_per_s_log.append(deg_per_s)
            pulse_deg_per_s_lpf = lowpass_first_order(pulse_deg_per_s_lpf, deg_per_s, dt, PULSE_SPEED_FILTER_TAU)
            if math.isfinite(pulse_deg_per_s_lpf):
                pulse_deg_per_s_filtered_log.append(pulse_deg_per_s_lpf)
                pulse_rpm_filtered_log.append(pulse_deg_per_s_lpf / 6.0)
            else:
                pulse_deg_per_s_filtered_log.append(float("nan"))
                pulse_rpm_filtered_log.append(float("nan"))
            pulse_deg_log.append(total_pulses * DEG_PER_STEP)
            cmd_rate_log.append(current_cmd_rate)
            time_log.append(ts - t0)
            last_sample_pulse_count = total_pulses
            last_sample_t = ts

    for d in delays:
        current_cmd_rate = 1.0 / (2.0 * d)
        queue_pulse(h, step_pin, d)
        total_pulses += 1
        sample_all(time.monotonic())

    time.sleep(min_delay * TX_BACKLOG)
    current_cmd_rate = 0.0
    sample_all(time.monotonic())

    out = {"t": np.array(time_log, dtype=float),
           "cmd_rate": np.array(cmd_rate_log, dtype=float),
           "pulse_rpm": np.array(pulse_rpm_log, dtype=float),
           "pulse_deg": np.array(pulse_deg_log, dtype=float),
           "pulse_deg_per_s": np.array(pulse_deg_per_s_log, dtype=float),
           "pulse_deg_per_s_filtered": np.array(pulse_deg_per_s_filtered_log, dtype=float),
           "pulse_rpm_filtered": np.array(pulse_rpm_filtered_log, dtype=float)}
    for ch, vals in enc_logs.items():
        out[f"enc_{ch}"] = np.array(vals, dtype=float)
    return out

# -------------------- S-curve + PID (closed-loop) --------------------
def move_stepper_scurve_with_pid(
    h, dir_pin, step_pin, total_steps, direction,
    min_delay, max_delay, accel_ratio=0.2,
    s_curve_gamma: float = 1.0,
    pid_gains=(2.0, 0.2, 0.0),  # (Kp, Ki, Kd) in (deg/s) per deg
    log_enc: bool = True, enc: EncoderMux = None, enc_channels=None,
    enc_sample_dt: float = ENC_SAMPLE_DT,
    decel_ratio=None,
    s_curve_gamma_decel=None,
    encoders_dict=None,
):
    if encoders_dict is None:
        encoders_dict = {}
    if (enc is None and not encoders_dict) or not enc_channels:
        return move_stepper_scurve_with_logging(
            h, dir_pin, step_pin, total_steps, direction, min_delay, max_delay,
            accel_ratio, s_curve_gamma, log_enc=False, enc=None, enc_channels=[], enc_sample_dt=enc_sample_dt,
            encoders_dict=encoders_dict
        )

    if min_delay >= max_delay:
        min_delay, max_delay = sorted([min_delay, max_delay])
    min_delay = max(min_delay, MIN_SAFE_DELAY)

    lgpio.gpio_write(h, dir_pin, direction)

    ff_delays, accel_steps, const_steps, decel_steps = build_delay_profile(
        total_steps=total_steps,
        min_delay=min_delay,
        max_delay=max_delay,
        accel_ratio=accel_ratio,
        decel_ratio=decel_ratio,
        gamma_accel=s_curve_gamma,
        gamma_decel=s_curve_gamma_decel,
    )

    pulses_per_rev = PULSES_PER_REV if math.isfinite(PULSES_PER_REV) and PULSES_PER_REV > 0 else float("nan")

    t0 = time.monotonic()
    last_sample_t = t0
    last_sample_pulse_count = 0
    total_pulses = 0
    time_log, cmd_rate_log = [], []
    pulse_rpm_log = []
    pulse_deg_log = []
    pulse_deg_per_s_log = []
    pulse_deg_per_s_filtered_log = []
    pulse_rpm_filtered_log = []
    pulse_deg_per_s_lpf = float("nan")
    enc_logs = {ch: [] for ch in enc_channels}
    
    # PID control logs
    pid_trim_log = []
    error_log = []
    target_angle_log = []
    measured_angle_log = []

    cmd_rate_ff = np.array([delay_to_rate(d) for d in ff_delays], dtype=float)
    cmd_ang_vel_ff = cmd_rate_ff * DEG_PER_STEP
    t_grid = np.cumsum(np.array(ff_delays, dtype=float))
    if len(t_grid) > 0:
        t_grid -= t_grid[0]
    cmd_angle_ff = np.zeros_like(t_grid)
    for i in range(1, len(t_grid)):
        dt = t_grid[i] - t_grid[i-1]
        cmd_angle_ff[i] = cmd_angle_ff[i-1] + 0.5 * (cmd_ang_vel_ff[i] + cmd_ang_vel_ff[i-1]) * dt

    Kp, Ki, Kd = pid_gains
    pid = PID(kp=Kp, ki=Ki, kd=Kd, out_min=-600.0, out_max=600.0, tau=0.01)
    last_t = t0
    
    # Track current PID state for logging
    current_trim = 0.0
    current_error = 0.0
    current_target = 0.0
    current_measured = 0.0

    def sample_all(ts):
        nonlocal last_sample_t, last_sample_pulse_count, pulse_deg_per_s_lpf
        if ts - last_sample_t >= enc_sample_dt:
            for ch in enc_channels:
                # Use encoder from dictionary if available, otherwise fall back to enc parameter
                encoder = encoders_dict.get(ch, enc)
                if encoder is not None:
                    # For GPIO encoder, update state before reading
                    if isinstance(encoder, GPIOEncoder):
                        encoder.update()
                    val = encoder.try_read_channel(ch)
                    enc_logs[ch].append(val)
            dt = ts - last_sample_t
            if dt <= 0.0:
                dt = 1e-6
            pulse_delta = total_pulses - last_sample_pulse_count
            rpm, _ = pulses_to_speed(pulse_delta, dt, pulses_per_rev)
            deg_per_s = (pulse_delta * DEG_PER_STEP) / dt
            if (not math.isfinite(rpm)) and math.isfinite(deg_per_s):
                rpm = deg_per_s / 6.0
            pulse_rpm_log.append(rpm)
            pulse_deg_per_s_log.append(deg_per_s)
            pulse_deg_per_s_lpf = lowpass_first_order(pulse_deg_per_s_lpf, deg_per_s, dt, PULSE_SPEED_FILTER_TAU)
            if math.isfinite(pulse_deg_per_s_lpf):
                pulse_deg_per_s_filtered_log.append(pulse_deg_per_s_lpf)
                pulse_rpm_filtered_log.append(pulse_deg_per_s_lpf / 6.0)
            else:
                pulse_deg_per_s_filtered_log.append(float("nan"))
                pulse_rpm_filtered_log.append(float("nan"))
            pulse_deg_log.append(total_pulses * DEG_PER_STEP)
            time_log.append(ts - t0)
            cmd_rate_log.append(current_rate_steps)
            
            # Log PID data
            pid_trim_log.append(current_trim)
            error_log.append(current_error)
            target_angle_log.append(current_target)
            measured_angle_log.append(current_measured)
            
            last_sample_pulse_count = total_pulses
            last_sample_t = ts

    fb_ch = enc_channels[0]
    fb_encoder = encoders_dict.get(fb_ch, enc)
    if fb_encoder is not None:
        if isinstance(fb_encoder, GPIOEncoder):
            fb_encoder.update()
        fb_encoder.select_channel(fb_ch)
        try:
            ang0 = fb_encoder.read_angle_deg()
        except Exception:
            ang0 = 0.0
    else:
        ang0 = 0.0
    target_offset = ang0
    current_rate_steps = delay_to_rate(ff_delays[0]) if ff_delays else 0.0

    for i, ff_d in enumerate(ff_delays):
        now = time.monotonic()
        dt = max(now - last_t, 1e-4)
        last_t = now

        target_ang = cmd_angle_ff[i] + target_offset
        current_target = target_ang

        if fb_encoder is not None:
            if isinstance(fb_encoder, GPIOEncoder):
                fb_encoder.update()
            fb_encoder.select_channel(fb_ch)
            try:
                ang_meas = fb_encoder.read_angle_deg()
            except Exception:
                ang_meas = float('nan')
        else:
            ang_meas = float('nan')

        current_measured = ang_meas
        
        if not np.isnan(ang_meas):
            e = target_ang - ang_meas
            current_error = e
            trim_deg_per_s = pid.update(e, dt, anti_windup_ref=0.0)
            current_trim = trim_deg_per_s
        else:
            current_error = float('nan')
            trim_deg_per_s = 0.0
            current_trim = 0.0

        ff_rate_steps = delay_to_rate(ff_d)
        trim_steps = trim_deg_per_s / max(DEG_PER_STEP, 1e-9)
        desired_rate_steps = max(ff_rate_steps + trim_steps, 1.0)

        d = rate_to_delay(desired_rate_steps, min_delay, max_delay)
        current_rate_steps = delay_to_rate(d)

        queue_pulse(h, step_pin, d)
        total_pulses += 1
        sample_all(time.monotonic())

    time.sleep(min_delay * TX_BACKLOG)
    current_cmd_rate = 0.0
    current_trim = 0.0
    sample_all(time.monotonic())

    out = {"t": np.array(time_log, dtype=float),
           "cmd_rate": np.array(cmd_rate_log, dtype=float),
           "pulse_rpm": np.array(pulse_rpm_log, dtype=float),
           "pulse_deg": np.array(pulse_deg_log, dtype=float),
           "pulse_deg_per_s": np.array(pulse_deg_per_s_log, dtype=float),
           "pulse_deg_per_s_filtered": np.array(pulse_deg_per_s_filtered_log, dtype=float),
           "pulse_rpm_filtered": np.array(pulse_rpm_filtered_log, dtype=float),
           "pid_trim": np.array(pid_trim_log, dtype=float),
           "pid_error": np.array(error_log, dtype=float),
           "target_angle": np.array(target_angle_log, dtype=float),
           "measured_angle": np.array(measured_angle_log, dtype=float)}
    for ch, vals in enc_logs.items():
        out[f"enc_{ch}"] = np.array(vals, dtype=float)
    return out

# -------------------- Main --------------------
def main():
    if lgpio is None:
        print("[ERROR] lgpio is not available in this environment.")
        return
    
    h = lgpio.gpiochip_open(0)
    
    # Initialize AS5600 magnetic encoder via I2C (GPIO 2,3 = SDA,SCL)
    gpio_enc = None
    try:
        gpio_enc = AS5600Encoder(bus_num=1, i2c_addr=0x36)
        test_angle = gpio_enc.read_angle_deg()
        magnet_ok, too_weak, too_strong = gpio_enc.get_magnet_status()
        agc = gpio_enc.get_agc_value()
        
        print(f"[INFO] AS5600 magnetic encoder initialized via I2C (GPIO 2=SDA, 3=SCL)")
        print(f"[INFO] Current angle: {test_angle:.2f} degrees")
        print(f"[INFO] I2C Address: 0x36")
        print(f"[INFO] Magnet status: {'✓ OK' if magnet_ok else '✗ NOT DETECTED'}")
        
        if too_weak:
            print(f"[WARN] Magnet too weak - move magnet closer to sensor (1-3mm)")
        elif too_strong:
            print(f"[WARN] Magnet too strong - move magnet farther from sensor")
        
        print(f"[INFO] AGC value: {agc} (optimal: 96-160, current: {'✓' if 96 <= agc <= 160 else '⚠'})")
        
        if not magnet_ok:
            print(f"[WARN] Magnet not detected! Encoder readings may be unreliable.")
            
    except Exception as e:
        print(f"[WARN] AS5600 encoder init failed: {e}")
        print(f"[INFO] Troubleshooting:")
        print(f"       1. Enable I2C: sudo raspi-config -> Interface Options -> I2C")
        print(f"       2. Check I2C devices: i2cdetect -y 1")
        print(f"       3. Install smbus: sudo apt-get install python3-smbus")
        print(f"       4. Check AS5600 wiring and magnet position")
        gpio_enc = None
    
    # Initialize I2C encoder (if available)
    try:
        enc = EncoderMux(bus_num=1, settle=0.002, retries=3, retry_wait=0.001) if smbus is not None else None
        candidates = [0, 1]
        working = enc.detect_working_channels(candidates) if enc is not None else []
        if enc is None or not working:
            print("[WARN] No I2C encoder channels responded.")
            enc = None
            working = []
        else:
            print("[INFO] Working I2C encoder channels:", working)
    except Exception as e:
        print(f"[WARN] I2C encoder init failed: {e}")
        enc = None
        working = []
    
    # Combine encoders: use GPIO encoder as primary if available, add I2C channels
    encoders = {}
    enc_channels = []
    
    if gpio_enc is not None:
        # GPIO encoder uses channel identifier "gpio"
        encoders["gpio"] = gpio_enc
        enc_channels.append("gpio")
    
    if enc is not None and working:
        # I2C encoder channels
        for ch in working:
            encoders[ch] = enc
            enc_channels.append(ch)
    
    if not enc_channels:
        print("[WARN] No encoders available - proceeding without encoder logging.")
        enc = None
        enc_channels = []
    else:
        print(f"[INFO] Total encoder channels available: {enc_channels}")
        # For compatibility, set enc to first available encoder
        enc = encoders[enc_channels[0]] if enc_channels else None
    
    for pin in (
        DIR_PIN_NAMA_17, STEP_PIN_NAMA_17, ENA_PIN_NAMA_17,
        DIR_PIN_NAMA_23, STEP_PIN_NAMA_23, ENA_PIN_NAMA_23,
        IN1_PIN, IN2_PIN, PWM_PIN
    ):
        lgpio.gpio_claim_output(h, pin)

    try:
        print("Commands:")
        print("  ACT e/r                              -> actuator extend/retract")
        print("  M1 f/b [steps] [accel] [gamma <shape>] [accel2 <decel>] [gamma2 <shape>] [speed <deg/s>|rpm <value>] [pid? Kp Ki Kd]  -> motor1")
        print("  M2 f/b [steps] [accel] [gamma <shape>] [accel2 <decel>] [gamma2 <shape>] [speed <deg/s>|rpm <value>] [pid? Kp Ki Kd]  -> motor2")
        print("  q                                    -> quit")

        while True:
            cmd = input(">> ").strip().lower().split()
            if not cmd:
                continue
            if cmd[0] == 'q':
                break

            if cmd[0] == 'act' and len(cmd) >= 2:
                if cmd[1] == 'e':
                    extend(h, 1.0)
                elif cmd[1] == 'r':
                    retract(h, 1.0)
                else:
                    print("Specify e (extend) or r (retract)")
                    continue
                time.sleep(ACT_TIME)
                stop_actuator(h)
                continue

            target_speed_deg = None
            s_curve_gamma = 1.0
            use_pid = False
            pid_gains = (2.0, 0.2, 0.0)

            def parse_motion_args(motor_name):
                nonlocal use_pid, pid_gains, target_speed_deg
                if len(cmd) < 2:
                    print("Invalid command.")
                    return None
                direction = 0 if cmd[1] == 'f' else 1
                steps = int(cmd[2]) if len(cmd) >= 3 else (STEPS_17 if motor_name == 'M1' else STEPS_23)
                accel_ratio = float(cmd[3]) if len(cmd) >= 4 else 0.2
                decel_ratio = None
                s_curve_gamma_decel = None
                s_gamma = 1.0
                idx = 4
                while idx < len(cmd):
                    token = cmd[idx]
                    # First, handle inline assignments like key=value
                    if '=' in token:
                        key, val = token.split('=', 1)
                        try:
                            v = float(val)
                        except Exception:
                            v = None
                        if key in ('gamma', 'g') and v is not None:
                            s_gamma = max(v, 0.1)
                            idx += 1
                            continue
                        if key in ('gamma2', 'gammad', 'gamma_d') and v is not None:
                            s_curve_gamma_decel = max(v, 0.1)
                            idx += 1
                            continue
                        if key in ('accel2', 'decel', 'decel_ratio') and v is not None:
                            decel_ratio = v
                            idx += 1
                            continue
                        if key == 'speed' and v is not None:
                            target_speed_deg = v
                            idx += 1
                            continue
                        if key == 'rpm' and v is not None:
                            target_speed_deg = v * 6.0
                            idx += 1
                            continue
                        # If not matched, fall through to keyword handling below
                    # helper: parse value that may be in the same token (e.g., gamma=0.5)
                    def _extract_value(next_idx):
                        # Case 1: token like 'key=1.2'
                        if '=' in token:
                            try:
                                return float(token.split('=', 1)[1]), idx + 1
                            except Exception:
                                return None, idx + 1
                        # Case 2: pattern 'key = 1.2'
                        if next_idx < len(cmd) and cmd[next_idx] == '=':
                            if next_idx + 1 < len(cmd):
                                try:
                                    return float(cmd[next_idx + 1]), next_idx + 2
                                except Exception:
                                    return None, next_idx + 2
                            return None, next_idx + 1
                        # Case 3: plain 'key 1.2'
                        if next_idx < len(cmd):
                            try:
                                return float(cmd[next_idx]), next_idx + 1
                            except Exception:
                                return None, next_idx + 1
                        return None, next_idx
                    if token == 'pid':
                        use_pid = True
                        if idx + 3 < len(cmd):
                            try:
                                Kp = float(cmd[idx + 1]); Ki = float(cmd[idx + 2]); Kd = float(cmd[idx + 3])
                                pid_gains = (Kp, Ki, Kd)
                            except Exception:
                                pass
                            idx += 4
                        else:
                            idx = len(cmd)
                    elif token in ('gamma', 'g'):
                        val, new_idx = _extract_value(idx + 1)
                        if val is not None:
                            s_gamma = max(val, 0.1)
                        else:
                            print("[WARN] Invalid gamma value; ignoring.")
                        idx = new_idx
                    elif token in ('gamma2', 'gammad', 'gamma_d') and idx + 1 < len(cmd):
                        val, new_idx = _extract_value(idx + 1)
                        if val is not None:
                            s_curve_gamma_decel = max(val, 0.1)
                        else:
                            print("[WARN] Invalid gamma2 value; ignoring.")
                        idx = new_idx
                    elif token in ('accel2', 'decel', 'decel_ratio') and idx + 1 < len(cmd):
                        val, new_idx = _extract_value(idx + 1)
                        if val is not None:
                            decel_ratio = val
                        else:
                            print("[WARN] Invalid accel2/decel value; ignoring.")
                        idx = new_idx
                    elif token == 'speed' and idx + 1 < len(cmd):
                        val, new_idx = _extract_value(idx + 1)
                        if val is not None:
                            target_speed_deg = val
                        else:
                            print("[WARN] Invalid speed value; ignoring.")
                        idx = new_idx
                    elif token == 'rpm' and idx + 1 < len(cmd):
                        val, new_idx = _extract_value(idx + 1)
                        if val is not None:
                            target_speed_deg = val * 6.0
                        else:
                            print("[WARN] Invalid RPM value; ignoring.")
                        idx = new_idx
                    else:
                        idx += 1
                return direction, steps, accel_ratio, s_gamma, decel_ratio, s_curve_gamma_decel

            achieved_speed_deg = None

            if cmd[0] == 'm1':
                parsed = parse_motion_args('M1')
                if not parsed:
                    continue
                direction, steps, accel_ratio, s_curve_gamma, decel_ratio, s_curve_gamma_decel = parsed
                name = 'M1'
                dir_pin, step_pin, ena_pin = DIR_PIN_NAMA_17, STEP_PIN_NAMA_17, ENA_PIN_NAMA_17
                min_delay, max_delay = MIN_DELAY_17, MAX_DELAY_17
            elif cmd[0] == 'm2':
                parsed = parse_motion_args('M2')
                if not parsed:
                    continue
                direction, steps, accel_ratio, s_curve_gamma, decel_ratio, s_curve_gamma_decel = parsed
                name = 'M2'
                dir_pin, step_pin, ena_pin = DIR_PIN_NAMA_23, STEP_PIN_NAMA_23, ENA_PIN_NAMA_23
                min_delay, max_delay = MIN_DELAY_23, MAX_DELAY_23
            else:
                print("Unknown command.")
                continue

            min_delay, max_delay, achieved_speed_deg = apply_speed_target_to_delays(
                min_delay, max_delay, target_speed_deg
            )
            speed_info = ""
            if target_speed_deg is not None:
                if achieved_speed_deg is not None:
                    speed_info = f", target_speed~={achieved_speed_deg:.1f} deg/s ({achieved_speed_deg/6.0:.1f} RPM)"
                else:
                    speed_info = ", target_speed ignored"

            extra = ""
            if decel_ratio is not None and decel_ratio != accel_ratio:
                extra += f", decel_ratio={decel_ratio}"
            if s_curve_gamma_decel is not None and s_curve_gamma_decel != s_curve_gamma:
                extra += f", gamma2={s_curve_gamma_decel}"
            print(f"[{name}] steps={steps}, accel_ratio={accel_ratio}, gamma={s_curve_gamma}{extra}, direction={direction}, PID={use_pid} gains={pid_gains if use_pid else '-'}{speed_info}")
            
            # Show which encoders will be used
            if use_pid and enc_channels:
                print(f"[INFO] PID control will use encoder: {enc_channels[0]}")
            if enc_channels:
                print(f"[INFO] Logging from encoders: {enc_channels}")
            
            # Check encoder status before movement
            initial_count = 0
            initial_angle = 0.0
            if gpio_enc is not None:
                initial_angle = gpio_enc.read_angle_deg()
                print(f"[DEBUG] Encoder before movement - Angle: {initial_angle:.2f} deg")

            enable_motor(h, ena_pin, True)
            if use_pid and enc_channels:
                logs = move_stepper_scurve_with_pid(
                    h, dir_pin, step_pin, steps, direction,
                    min_delay, max_delay, accel_ratio=accel_ratio,
                    s_curve_gamma=s_curve_gamma,
                    pid_gains=pid_gains,
                    log_enc=True, enc=enc, enc_channels=enc_channels, enc_sample_dt=ENC_SAMPLE_DT,
                    decel_ratio=decel_ratio, s_curve_gamma_decel=s_curve_gamma_decel,
                    encoders_dict=encoders
                )
            else:
                logs = move_stepper_scurve_with_logging(
                    h, dir_pin, step_pin, steps, direction,
                    min_delay, max_delay, accel_ratio=accel_ratio,
                    s_curve_gamma=s_curve_gamma,
                    log_enc=bool(enc_channels), enc=enc, enc_channels=enc_channels, enc_sample_dt=ENC_SAMPLE_DT,
                    decel_ratio=decel_ratio, s_curve_gamma_decel=s_curve_gamma_decel,
                    encoders_dict=encoders
                )
            enable_motor(h, ena_pin, False)
            
            # Check encoder status after movement
            if gpio_enc is not None:
                final_angle = gpio_enc.read_angle_deg()
                angle_change = final_angle - initial_angle
                # Handle wrap-around for absolute encoder
                if angle_change > 180:
                    angle_change -= 360
                elif angle_change < -180:
                    angle_change += 360
                print(f"[DEBUG] Encoder after movement - Angle: {final_angle:.2f} deg")
                print(f"[DEBUG] Encoder change - Angle: {angle_change:+.2f} deg")
                if abs(angle_change) < 0.5:
                    print("[WARN] Encoder angle did not change significantly! Check:")
                    print("       - AS5600 magnet position (should be 1-3mm from sensor)")
                    print("       - Motor is actually rotating")
                    print("       - Encoder is mechanically coupled to motor shaft")

            if logs is None or 't' not in logs or len(logs['t']) < 3:
                print("[INFO] No data to plot.")
                continue

            # ===== Encoder Debug Information =====
            print("\n" + "="*60)
            print("ENCODER DEBUG INFORMATION")
            print("="*60)
            
            encoder_found = False
            for ch in enc_channels:
                enc_key = f"enc_{ch}"
                if enc_key in logs:
                    enc_data = logs[enc_key]
                    valid_count = np.sum(~np.isnan(enc_data))
                    print(f"[DEBUG] Encoder '{ch}':")
                    print(f"  - Samples: {len(enc_data)}")
                    print(f"  - Valid: {valid_count}/{len(enc_data)} ({100*valid_count/len(enc_data):.1f}%)")
                    print(f"  - Min: {np.nanmin(enc_data):.2f} deg, Max: {np.nanmax(enc_data):.2f} deg")
                    print(f"  - Range: {np.nanmax(enc_data) - np.nanmin(enc_data):.2f} deg")
                    if len(enc_data) >= 10:
                        print(f"  - First 10 values: {enc_data[:10]}")
                    encoder_found = True
                else:
                    print(f"[WARN] Encoder '{ch}' not in logs")
            
            if not encoder_found:
                print("[ERROR] No encoder data in logs!")
                print("[ERROR] AS5600 encoder (I2C via GPIO 2,3) may not be working properly.")
                print("[ERROR] Please check:")
                print("  1. I2C is enabled: sudo raspi-config -> Interface Options -> I2C")
                print("  2. I2C devices detected: i2cdetect -y 1 (should show 0x36)")
                print("  3. AS5600 power supply (VCC=3.3V or 5V, GND)")
                print("  4. Magnet is positioned correctly (1-3mm from sensor)")
                print("  5. Motor/encoder shaft coupling is secure")
            
            print("="*60 + "\n")
            
            t = logs['t']
            cmd_rate = logs.get('cmd_rate', None)
            cmd_ang_vel = cmd_rate * DEG_PER_STEP if cmd_rate is not None else None
            cmd_angle = cumtrapz(cmd_ang_vel, t) if cmd_ang_vel is not None else None
            pulse_deg = logs.get('pulse_deg', None)
            pulse_deg_per_s = logs.get('pulse_deg_per_s', None)
            pulse_deg_per_s_filtered = logs.get('pulse_deg_per_s_filtered', None)
            pulse_rpm = logs.get('pulse_rpm', None)
            pulse_rpm_filtered = logs.get('pulse_rpm_filtered', None)

            csv_pulse_vel = None
            if pulse_deg_per_s_filtered is not None and len(pulse_deg_per_s_filtered) == len(t):
                csv_pulse_vel = pulse_deg_per_s_filtered
            elif pulse_deg_per_s is not None and len(pulse_deg_per_s) == len(t):
                csv_pulse_vel = pulse_deg_per_s

            # Prepare PID data for CSV (if using PID control)
            csv_pid_error = logs.get('pid_error', None) if use_pid else None
            csv_pid_trim = logs.get('pid_trim', None) if use_pid else None
            csv_target_angle = logs.get('target_angle', None) if use_pid else None
            csv_measured_angle = logs.get('measured_angle', None) if use_pid else None

            csv_path = export_motion_csv(
                t=t,
                cmd_angle=cmd_angle,
                pulse_deg=pulse_deg,
                cmd_ang_vel=cmd_ang_vel,
                pulse_vel=csv_pulse_vel,
                filename_base=f"{name.lower()}_{'pid' if use_pid else 'open'}",
                pid_error=csv_pid_error,
                pid_trim=csv_pid_trim,
                target_angle=csv_target_angle,
                measured_angle=csv_measured_angle
            )
            if csv_path:
                print(f"[INFO] Motion CSV saved: {csv_path}")
                if plot_csv_results is not None:
                    try:
                        plot_csv_results(csv_path)
                    except Exception as exc:
                        print(f"[WARN] Plotting CSV failed: {exc}")

            # ===== PID Performance Analysis =====
            if use_pid:
                pid_error = logs.get('pid_error', None)
                pid_trim = logs.get('pid_trim', None)
                target_angle = logs.get('target_angle', None)
                measured_angle = logs.get('measured_angle', None)
                
                # Calculate PID performance metrics
                if target_angle is not None and measured_angle is not None:
                    metrics = calculate_pid_metrics(t, target_angle, measured_angle, settling_tolerance=0.02)
                    if metrics:
                        print("\n" + "="*60)
                        print("PID PERFORMANCE METRICS")
                        print("="*60)
                        print(f"RMS Error:              {metrics['rms_error']:.4f} deg")
                        print(f"Max Error:              {metrics['max_error']:.4f} deg")
                        print(f"Mean Absolute Error:    {metrics['mae']:.4f} deg")
                        print(f"Steady-State Error:     {metrics['steady_state_error']:.4f} deg (abs: {metrics['steady_state_error_abs']:.4f})")
                        print(f"Overshoot:              {metrics['overshoot_value']:.4f} deg ({metrics['overshoot_percent']:.2f}%)")
                        if metrics['settling_time'] is not None:
                            print(f"Settling Time (2%):     {metrics['settling_time']:.3f} s")
                        else:
                            print(f"Settling Time (2%):     Not achieved")
                        if metrics['rise_time'] is not None:
                            print(f"Rise Time (10%-90%):    {metrics['rise_time']:.3f} s")
                        print(f"Final Target:           {metrics['final_target']:.4f} deg")
                        print(f"Final Measured:         {metrics['final_measured']:.4f} deg")
                        print("="*60 + "\n")
                        
                        # Quality assessment
                        quality_issues = []
                        if metrics['overshoot_percent'] > 10:
                            quality_issues.append("⚠️  High overshoot (>10%) - Consider reducing Kp or increasing Kd")
                        if abs(metrics['steady_state_error_abs']) > 0.5:
                            quality_issues.append("⚠️  High steady-state error - Consider increasing Ki")
                        if metrics['settling_time'] is None:
                            quality_issues.append("⚠️  System did not settle - Tune gains or increase test duration")
                        
                        if quality_issues:
                            print("PID TUNING RECOMMENDATIONS:")
                            for issue in quality_issues:
                                print(f"  {issue}")
                            print()
                        else:
                            print("✅ PID performance is good!\n")
                
                # Plot PID-specific data
                if pid_error is not None and len(pid_error) == len(t):
                    # Error plot
                    plot_time_series(t, [pid_error], ["Position Error (deg)"], 
                                    f"{name} PID Error", "Error (deg)", f"{name.lower()}_pid_error")
                
                if pid_trim is not None and len(pid_trim) == len(t):
                    # Control output (trim) plot
                    plot_time_series(t, [pid_trim], ["PID Output (deg/s)"], 
                                    f"{name} PID Control Output", "Control Output (deg/s)", f"{name.lower()}_pid_output")
                
                if target_angle is not None and measured_angle is not None and len(target_angle) == len(t):
                    # Target vs Measured overlay
                    plot_time_series(t, [target_angle, measured_angle], 
                                    ["Target Angle", "Measured Angle"],
                                    f"{name} PID Tracking Performance", "Angle (deg)", f"{name.lower()}_pid_tracking")

            cmd_rpm = None
            if cmd_rate is not None and math.isfinite(PULSES_PER_REV) and PULSES_PER_REV > 0:
                cmd_rpm = cmd_rate * (60.0 / PULSES_PER_REV)

            rpm_for_stats = None
            rpm_stats_label = ""
            if pulse_rpm_filtered is not None and len(pulse_rpm_filtered):
                rpm_for_stats = pulse_rpm_filtered
                rpm_stats_label = "filtered"
            elif pulse_rpm is not None and len(pulse_rpm):
                rpm_for_stats = pulse_rpm
                rpm_stats_label = "raw"

            if rpm_for_stats is not None and len(rpm_for_stats):
                finite_mask = np.isfinite(rpm_for_stats)
                if finite_mask.any():
                    rpm_values = rpm_for_stats[finite_mask]
                    avg_rpm = float(np.mean(rpm_values))
                    max_rpm = float(np.max(rpm_values))
                    print(f"[INFO] Pulse-based RPM ({rpm_stats_label}) avg={avg_rpm:.2f}, max={max_rpm:.2f}")
                else:
                    print(f"[WARN] Pulse-based RPM ({rpm_stats_label}) contains only NaNs.")

            # Summaries of what has already been plotted; prevents duplicate command-only charts later.
            pulse_vel_plot = False
            pulse_pos_plot = False

            # Compare commanded angular velocity vs. pulse-derived velocities (raw and filtered).
            has_pulse_vel = pulse_deg_per_s is not None and len(pulse_deg_per_s) == len(t)
            has_pulse_vel_f = pulse_deg_per_s_filtered is not None and len(pulse_deg_per_s_filtered) == len(t)
            if has_pulse_vel or has_pulse_vel_f:
                vel_series = []
                vel_labels = []
                if cmd_ang_vel is not None:
                    vel_series.append(cmd_ang_vel)
                    vel_labels.append("commanded ω [deg/s]")
                if has_pulse_vel:
                    vel_series.append(pulse_deg_per_s)
                    vel_labels.append("pulse ω raw [deg/s]")
                if has_pulse_vel_f:
                    vel_series.append(pulse_deg_per_s_filtered)
                    vel_labels.append("pulse ω LPF [deg/s]")
                plot_time_series(t, vel_series, vel_labels, "Pulse-derived Angular Velocity", "Angular velocity (deg/s)", "pulse_speed_deg")
                pulse_vel_plot = True

            # Overlay commanded vs. pulse-derived RPM traces (raw / filtered).
            has_pulse_rpm = pulse_rpm is not None and len(pulse_rpm) == len(t)
            has_pulse_rpm_f = pulse_rpm_filtered is not None and len(pulse_rpm_filtered) == len(t)
            if has_pulse_rpm or has_pulse_rpm_f:
                rpm_series = []
                rpm_labels = []
                if cmd_rpm is not None:
                    rpm_series.append(cmd_rpm)
                    rpm_labels.append("commanded RPM")
                if has_pulse_rpm:
                    rpm_series.append(pulse_rpm)
                    rpm_labels.append("pulse RPM raw")
                if has_pulse_rpm_f:
                    rpm_series.append(pulse_rpm_filtered)
                    rpm_labels.append("pulse RPM LPF")
                plot_time_series(t, rpm_series, rpm_labels, "Pulse-derived Speed (RPM)", "Speed (RPM)", "pulse_speed_rpm")

            # Compare commanded angle to pulse-integrated angle for displacement accuracy.
            if pulse_deg is not None and len(pulse_deg) == len(t):
                pos_series = []
                pos_labels = []
                if cmd_angle is not None:
                    pos_series.append(cmd_angle)
                    pos_labels.append("commanded θ [deg]")
                    pulse_deg_aligned = pulse_deg - pulse_deg[0] + cmd_angle[0]
                else:
                    pulse_deg_aligned = pulse_deg
                pos_series.append(pulse_deg_aligned)
                pos_labels.append("pulse θ [deg]")
                plot_time_series(t, pos_series, pos_labels, "Pulse-derived Angle", "Angle (deg)", "pulse_angle_deg")
                pulse_pos_plot = True

            usable = []
            if enc_channels:
                for ch in enc_channels:
                    key = f"enc_{ch}"
                    y = logs.get(key, None)
                    if y is None or len(y) != len(t):
                        continue
                    nan_ratio = np.isnan(y).mean()
                    if nan_ratio < 0.3:
                        usable.append(ch)
                    else:
                        print(f"[WARN] ch{ch} has too many NaNs ({nan_ratio:.0%}); skipping plot.")

            # Minimal RPM plot (raw) even if filtered version missing.
            if pulse_rpm is not None and len(pulse_rpm) == len(t):
                rpm_series = [pulse_rpm]
                rpm_labels = ["pulse RPM"]
                if cmd_rpm is not None:
                    rpm_series.append(cmd_rpm)
                    rpm_labels.append("commanded RPM")
                plot_time_series(t, rpm_series, rpm_labels, "Pulse-derived Speed", "Speed (RPM)", "pulse_speed_rpm")

            # Plots
            if not usable:
                # No encoder data available: fall back to command-only plots (unless already shown).
                if cmd_rate is not None:
                    if cmd_ang_vel is not None and not pulse_vel_plot:
                        plot_time_series(t, [cmd_ang_vel], ["commanded ω"], "Commanded Angular Velocity", "Angular velocity (deg/s)", "cmd_angular_velocity")
                        if cmd_angle is not None and not pulse_pos_plot:
                            plot_time_series(t, [cmd_angle], ["commanded θ"], "Commanded Angle (integrated)", "Angle (deg)", "cmd_angle")
                    plot_time_series(t, [cmd_rate], ["cmd step rate"], "Commanded Step Rate", "Steps per second", "cmd_step_rate")
                continue

            for ch in usable:
                key = f"enc_{ch}"
                ang_raw = logs[key]
                mask = ~np.isnan(ang_raw)
                ang_filled = ang_raw.copy()
                if not mask.all():
                    valid_t = t[mask]
                    valid_y = ang_raw[mask]
                    ang_filled = np.interp(t, valid_t, valid_y)
                ang_unwrapped = unwrap_deg(ang_filled)
                vel = finite_diff(ang_unwrapped, t)
                acc = finite_diff(vel, t)

                # Plot encoder angles against commands when available.
                if cmd_angle is not None:
                    cmd_angle_aligned = cmd_angle - cmd_angle[0] + ang_unwrapped[0]
                    plot_time_series(t, [ang_unwrapped, cmd_angle_aligned], [f"ch{ch} measured θ", "commanded θ"], f"Encoder ch{ch} Angle", "Angle (deg)", f"ch{ch}_angle_overlay")
                else:
                    plot_time_series(t, [ang_unwrapped], [f"ch{ch} measured θ"], f"Encoder ch{ch} Angle", "Angle (deg)", f"ch{ch}_angle")

                # Likewise compare encoder velocity and acceleration.
                if cmd_ang_vel is not None:
                    plot_time_series(t, [vel, cmd_ang_vel], [f"ch{ch} measured ω", "commanded ω"], f"Encoder ch{ch} Angular Velocity", "Angular velocity (deg/s)", f"ch{ch}_angular_velocity_overlay")
                else:
                    plot_time_series(t, [vel], [f"ch{ch} measured ω"], f"Encoder ch{ch} Angular Velocity", "Angular velocity (deg/s)", f"ch{ch}_angular_velocity")

                plot_time_series(t, [acc], [f"ch{ch} measured α"], f"Encoder ch{ch} Angular Acceleration", "Angular acceleration (deg/s^2)", f"ch{ch}_angular_acceleration")

    except KeyboardInterrupt:
        print("\n[Interrupted]")
    finally:
        try:
            stop_actuator(h)
        except Exception:
            pass
        try:
            if lgpio is not None:
                enable_motor(h, ENA_PIN_NAMA_17, False)
                enable_motor(h, ENA_PIN_NAMA_23, False)
                lgpio.gpiochip_close(h)
        except Exception:
            pass
        print("GPIO released.")

if __name__ == "__main__":
    main()

