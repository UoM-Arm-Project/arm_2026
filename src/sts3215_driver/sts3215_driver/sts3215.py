"""STS3215 servo bus driver built on the official FTServo Python SDK.

ROS-agnostic. Owns the serial port and exposes a human-readable API in raw
servo units: positions in steps (0-4095 per revolution) and speeds in
signed steps/second. Physical units (volts, degrees C, mA, percent) are
converted where the register definition demands it.

The SDK's ``sms_sts`` class provides the primitives; this wrapper adds
thread-safe access, named methods for every register we use, and a broadcast
sync write for wheel-mode speeds so callers never touch register addresses.

Register reference: STS3215 memory map (commanderfun/STS3215
REGISTER_REFERENCE.md, verified against firmware 3.10).
"""

import threading

from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS

STEPS_PER_REV = 4096
STEPS_PER_RAD = STEPS_PER_REV / (2.0 * 3.141592653589793)
SPEED_FIELD_MAX = 32767  # protocol sign-magnitude field, not a motion limit

# --- EEPROM registers (non-volatile; write with torque off) ---
REG_ID = 5
REG_BAUD_RATE = 6
REG_MIN_ANGLE_LIMIT = 9       # 2 bytes
REG_MAX_ANGLE_LIMIT = 11      # 2 bytes
REG_MAX_TEMP_LIMIT = 13
REG_MAX_VOLTAGE_LIMIT = 14
REG_MIN_VOLTAGE_LIMIT = 15
REG_MAX_TORQUE_LIMIT = 16     # 2 bytes, 0-1000
REG_P_COEFFICIENT = 21
REG_D_COEFFICIENT = 22
REG_I_COEFFICIENT = 23
REG_MIN_STARTUP_FORCE = 24    # 2 bytes, 0-1000
REG_CW_DEAD_ZONE = 26
REG_CCW_DEAD_ZONE = 27
REG_PROTECTION_CURRENT = 28   # 2 bytes, x6.5 mA
REG_POSITION_OFFSET = 31      # 2 bytes
REG_MODE = 33
REG_PROTECTION_TORQUE = 34
REG_PROTECTION_TIME = 35
REG_OVERLOAD_TORQUE = 36

# --- RAM registers (volatile) ---
REG_TORQUE_ENABLE = 40
REG_ACCELERATION = 41         # 0 = instant (firmware ramps at its own limit)
REG_GOAL_POSITION = 42        # 2 bytes
REG_GOAL_SPEED = 46           # 2 bytes, 0 = max speed
REG_TORQUE_LIMIT = 48         # 2 bytes, 0-1000
REG_LOCK = 55
REG_PRESENT_POSITION = 56     # 2 bytes
REG_PRESENT_SPEED = 58        # 2 bytes
REG_PRESENT_LOAD = 60         # 2 bytes
REG_PRESENT_VOLTAGE = 62      # x0.1 V
REG_PRESENT_TEMPERATURE = 63  # deg C
REG_SERVO_STATUS = 65         # error bitmask
REG_MOVING = 66
REG_PRESENT_CURRENT = 69      # 2 bytes, x6.5 mA

MODE_SERVO = 0   # position control
MODE_WHEEL = 1   # constant speed (velocity) control
MODE_PWM = 2     # open-loop PWM
MODE_STEP = 3    # step servo


class STS3215Bus:
    """Thread-safe handle on a chain of STS3215 servos on one serial port.

    Every public method serialises bus access on an internal lock, so a
    command callback and a feedback timer can run on different threads.
    """

    def __init__(self, port, baudrate=1_000_000, servo_ids=(1, 2, 3, 4, 5, 6)):
        self.port = PortHandler(port)
        self.port.baudrate = baudrate
        self.pk = sms_sts(self.port)
        self.servo_ids = list(servo_ids)
        self._lock = threading.Lock()

    # --- lifecycle -------------------------------------------------------
    def connect(self):
        """Open the serial port at the configured baud rate."""
        if not self.port.openPort():
            return False
        return self.port.setBaudRate(self.port.baudrate)

    def close(self):
        self.port.closePort()

    def ping(self, servo_id):
        """Return the model number or -1 if the servo does not answer."""
        with self._lock:
            model, result, _ = self.pk.ping(servo_id)
        return model if result == COMM_SUCCESS else -1

    def ping_all(self):
        """Return {id: model_number} for servos that answered (-1 otherwise)."""
        return {i: self.ping(i) for i in self.servo_ids}

    # --- generic register access (escape hatch) ---------------------------
    # The private variants assume the caller already holds _lock.
    def _read_u8(self, servo_id, register):
        value, result, _ = self.pk.read1ByteTxRx(servo_id, register)
        return value if result == COMM_SUCCESS else -1

    def _read_u16(self, servo_id, register):
        value, result, _ = self.pk.read2ByteTxRx(servo_id, register)
        return value if result == COMM_SUCCESS else -1

    def _write_u8(self, servo_id, register, value):
        return self.pk.write1ByteTxRx(servo_id, register, int(value))

    def _write_u16(self, servo_id, register, value):
        return self.pk.write2ByteTxRx(servo_id, register, int(value))

    def read_u8(self, servo_id, register):
        with self._lock:
            return self._read_u8(servo_id, register)

    def read_u16(self, servo_id, register):
        with self._lock:
            return self._read_u16(servo_id, register)

    def write_u8(self, servo_id, register, value):
        with self._lock:
            return self._write_u8(servo_id, register, value)

    def write_u16(self, servo_id, register, value):
        with self._lock:
            return self._write_u16(servo_id, register, value)

    # --- torque -----------------------------------------------------------
    def enable_torque(self, servo_ids=None):
        """Energise the servos so they hold and track commands."""
        ids = servo_ids or self.servo_ids
        with self._lock:
            for i in ids:
                self._write_u8(i, REG_TORQUE_ENABLE, 1)

    def disable_torque(self, servo_ids=None):
        """De-energise the servos; they go limp but still report position."""
        ids = servo_ids or self.servo_ids
        with self._lock:
            for i in ids:
                self._write_u8(i, REG_TORQUE_ENABLE, 0)

    def set_torque_limit(self, servo_id, percent_x10):
        """Limit peak torque, 0-1000 where 1000 = 100% of maximum."""
        return self.write_u16(servo_id, REG_TORQUE_LIMIT, int(percent_x10))

    # --- operating mode -----------------------------------------------------
    def set_mode(self, servo_id, mode):
        """MODE_SERVO (0), MODE_WHEEL (1), MODE_PWM (2) or MODE_STEP (3)."""
        return self.write_u8(servo_id, REG_MODE, mode)

    def get_mode(self, servo_id):
        return self.read_u8(servo_id, REG_MODE)

    def set_wheel_mode(self, servo_ids=None):
        """Switch servos to velocity (wheel) mode with torque enabled."""
        ids = servo_ids or self.servo_ids
        with self._lock:
            for i in ids:
                self._write_u8(i, REG_MODE, MODE_WHEEL)
                self._write_u8(i, REG_TORQUE_ENABLE, 1)

    def set_servo_mode(self, servo_ids=None):
        """Switch servos back to position mode."""
        ids = servo_ids or self.servo_ids
        with self._lock:
            for i in ids:
                self._write_u8(i, REG_MODE, MODE_SERVO)

    # --- velocity commands (wheel mode) ------------------------------------
    def set_speed(self, servo_id, speed, acc=0):
        """Write a signed wheel-mode speed in steps/s to one servo.

        ``acc`` 0 lets the firmware ramp at its own limit; 1-254 softens it.
        """
        with self._lock:
            return self.pk.WriteSpec(servo_id, int(speed), int(acc))

    def set_speeds(self, speeds, acc=0):
        """Broadcast signed speeds (steps/s) to all servos in one packet.

        ``speeds`` maps servo_id -> steps/s. Missing ids are not commanded.
        """
        if not speeds:
            return True
        with self._lock:
            g = self.pk.groupSyncWrite
            g.clearParam()
            for servo_id, speed in speeds.items():
                s = int(max(-SPEED_FIELD_MAX, min(SPEED_FIELD_MAX, speed)))
                s = self.pk.scs_toscs(s, 15)
                txpacket = [int(acc), 0, 0, 0, 0,
                            self.pk.scs_lobyte(s), self.pk.scs_hibyte(s)]
                if not g.addParam(servo_id, txpacket):
                    return False
            g.txPacket()
            return True

    def stop_all(self, acc=0):
        """Command zero speed to every servo."""
        self.set_speeds({i: 0 for i in self.servo_ids}, acc)

    # --- position commands (servo mode) ------------------------------------
    def set_position(self, servo_id, position, speed=0, acc=0):
        """Move a servo to ``position`` steps (0-4095) in servo mode.

        ``speed`` is a magnitude limit: 0 = max, 1-3400 steps/s. ``acc`` as
        for set_speed.
        """
        with self._lock:
            return self.pk.WritePosEx(servo_id, int(position),
                                      int(speed), int(acc))

    def set_acceleration(self, servo_id, acc):
        """Acceleration setting, 0 = instant, 1-254 = smoother ramp."""
        return self.write_u8(servo_id, REG_ACCELERATION, int(acc))

    # --- feedback -----------------------------------------------------------
    def _read_pos_speed(self, servo_id):
        pos, spd, result, _ = self.pk.ReadPosSpeed(servo_id)
        if result != COMM_SUCCESS:
            return -1, -1
        return pos, spd

    def read_position(self, servo_id):
        """Current position in steps (0-4095), or -1 on bus failure."""
        with self._lock:
            return self._read_pos_speed(servo_id)[0]

    def read_speed(self, servo_id):
        """Current signed speed in steps/s, or -1 on bus failure."""
        with self._lock:
            return self._read_pos_speed(servo_id)[1]

    def read_pos_speed(self, servo_id):
        """(position_steps, speed_steps_s); (-1, -1) on bus failure."""
        with self._lock:
            return self._read_pos_speed(servo_id)

    def read_all(self, servo_ids=None):
        """{id: (pos, speed)} for the given ids (default: all)."""
        ids = servo_ids or self.servo_ids
        with self._lock:
            return {i: self._read_pos_speed(i) for i in ids}

    def read_load(self, servo_id):
        """Current load, 0-1000 with direction in bit 10 of the raw value."""
        return self.read_u16(servo_id, REG_PRESENT_LOAD)

    def read_voltage(self, servo_id):
        """Bus voltage in volts (register x 0.1 V)."""
        raw = self.read_u8(servo_id, REG_PRESENT_VOLTAGE)
        return raw * 0.1 if raw >= 0 else -1.0

    def read_temperature(self, servo_id):
        """Motor temperature in degrees Celsius."""
        return self.read_u8(servo_id, REG_PRESENT_TEMPERATURE)

    def read_current(self, servo_id):
        """Motor current in milliamps (register x 6.5 mA)."""
        raw = self.read_u16(servo_id, REG_PRESENT_CURRENT)
        return raw * 6.5 if raw >= 0 else -1.0

    def read_status(self, servo_id):
        """Error bitmask: bit0 voltage, 1 sensor, 2 temp, 3 current, 5 overload."""
        return self.read_u8(servo_id, REG_SERVO_STATUS)

    def is_moving(self, servo_id):
        """True while the servo output shaft is turning."""
        with self._lock:
            moving, result, _ = self.pk.ReadMoving(servo_id)
        return bool(moving) if result == COMM_SUCCESS else False

    # --- EEPROM tuning (write with torque off; see datasheet) ---------------
    def set_pid(self, servo_id, p=None, d=None, i=None):
        """Set position-loop PID coefficients (P at 21, D at 22, I at 23).

        Pass None to leave a coefficient untouched. Factory defaults are
        P=32, D=32, I=0.
        """
        with self._lock:
            if p is not None:
                self._write_u8(servo_id, REG_P_COEFFICIENT, p)
            if d is not None:
                self._write_u8(servo_id, REG_D_COEFFICIENT, d)
            if i is not None:
                self._write_u8(servo_id, REG_I_COEFFICIENT, i)

    def get_pid(self, servo_id):
        """Return (p, d, i) coefficients."""
        return (self.read_u8(servo_id, REG_P_COEFFICIENT),
                self.read_u8(servo_id, REG_D_COEFFICIENT),
                self.read_u8(servo_id, REG_I_COEFFICIENT))

    def set_angle_limits(self, servo_id, min_steps, max_steps):
        """Position limits (0-4095) for servo mode. Ignored in wheel mode."""
        with self._lock:
            self._write_u16(servo_id, REG_MIN_ANGLE_LIMIT, min_steps)
            self._write_u16(servo_id, REG_MAX_ANGLE_LIMIT, max_steps)

    def get_angle_limits(self, servo_id):
        return (self.read_u16(servo_id, REG_MIN_ANGLE_LIMIT),
                self.read_u16(servo_id, REG_MAX_ANGLE_LIMIT))

    def set_max_torque(self, servo_id, percent_x10):
        """EEPROM torque ceiling, 0-1000 where 1000 = 100%."""
        return self.write_u16(servo_id, REG_MAX_TORQUE_LIMIT, percent_x10)

    def set_dead_zone(self, servo_id, cw_steps, ccw_steps):
        """Dead zone widths around the goal position, in steps."""
        with self._lock:
            self._write_u8(servo_id, REG_CW_DEAD_ZONE, cw_steps)
            self._write_u8(servo_id, REG_CCW_DEAD_ZONE, ccw_steps)

    def set_min_startup_force(self, servo_id, percent_x10):
        """Minimum torque to overcome stiction, 0-1000."""
        return self.write_u16(servo_id, REG_MIN_STARTUP_FORCE, percent_x10)

    def set_position_offset(self, servo_id, offset_steps):
        """Zero-point calibration offset in steps (factory ~85)."""
        return self.write_u16(servo_id, REG_POSITION_OFFSET, offset_steps)

    def set_servo_id(self, servo_id, new_id):
        """Renumber a servo (EEPROM). Do this one servo at a time."""
        return self.write_u8(servo_id, REG_ID, new_id)

    def unlock_eeprom(self, servo_id):
        """Allow EEPROM writes to persist across power cycles."""
        with self._lock:
            return self.pk.unLockEprom(servo_id)

    def lock_eeprom(self, servo_id):
        """Freeze EEPROM so writes are volatile (default behaviour)."""
        with self._lock:
            return self.pk.LockEprom(servo_id)
