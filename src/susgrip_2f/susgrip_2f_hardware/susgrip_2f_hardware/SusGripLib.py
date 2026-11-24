#!/usr/bin/env python3
"""
SusGrip 2F Driver Library
Updated to match Apicoo Universal Console Protocol (Firmware v2.x)

Register Map:
  Holding Registers (Write, FC 16 / FC 06):
    SET_EMCY     = 0   (0 = normal, 65535 = emergency stop)
    SET_POSITION = 1   (Target position in mm)
    SET_SPEED    = 2   (Target speed in mm/s)
    SET_ACCEL    = 3   (Target acceleration in mm/s^2)
    SET_FORCE    = 4   (Target force in %)
    CONFIG_MODE  = 257 (1 = activate/enable)

  Input Registers (Read, FC 04):
    0: INFO     (0x562f)
    1: POSITION (Actual position in mm, signed 16)
    2: SPEED    (Actual speed in mm/s)
    3: RMS      (RMS current / force)
    4: ERROR    (Error code)
    5: MOTION   (1 = in motion, 0 = stopped)
    6: OBJ      (1 = object detected, 0 = none)
"""

import sys
import os
import time
import threading

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pymodbus.client import ModbusSerialClient as ModbusClient

class SusGrip2F:
    def __init__(self, id=2, timeout=1.0, *args, **kwargs):
        port = kwargs.get('port')
        if not port:
            raise ValueError('A serial port must be supplied, for example /dev/ttyUSB0.')
        if not os.path.exists(port):
            raise FileNotFoundError(f'Serial port {port} does not exist.')
        if not os.access(port, os.R_OK | os.W_OK):
            raise PermissionError(
                f'No read/write access to {port}. Run: sudo chmod 666 {port}'
            )
        self.gripper_rtu = ModbusClient(
            port=port,
            baudrate=115200,
            parity='N',
            stopbits=1,
            bytesize=8,
            handle_local_echo=False,
            timeout=timeout
        )
        self._lock = threading.Lock()
        self.is_connect = False
        self.id = id
        self.timeout = timeout
        self.data = []
        self.last_read_error = None
        self.last_write_error = None

    # --- Robust Modbus wrappers adapting across all pymodbus versions ---
    def _write_register(self, address, value):
        with self._lock:
            for kw in [{'device_id': self.id}, {'slave': self.id}, {'unit': self.id}, {}]:
                try:
                    return self.gripper_rtu.write_register(address, value, **kw)
                except TypeError as te:
                    if 'unexpected keyword argument' in str(te):
                        continue
                    raise
            return None

    def _write_registers(self, address, values):
        with self._lock:
            for kw in [{'device_id': self.id}, {'slave': self.id}, {'unit': self.id}, {}]:
                try:
                    return self.gripper_rtu.write_registers(address, values, **kw)
                except TypeError as te:
                    if 'unexpected keyword argument' in str(te):
                        continue
                    raise
            return None

    def _read_input_registers(self, address, count):
        with self._lock:
            for kw in [{'device_id': self.id}, {'slave': self.id}, {'unit': self.id}, {}]:
                try:
                    return self.gripper_rtu.read_input_registers(address, count=count, **kw)
                except TypeError as te:
                    if 'unexpected keyword argument' in str(te):
                        continue
                    raise
            return None

    def _read_holding_registers(self, address, count):
        with self._lock:
            for kw in [{'device_id': self.id}, {'slave': self.id}, {'unit': self.id}, {}]:
                try:
                    return self.gripper_rtu.read_holding_registers(address, count=count, **kw)
                except TypeError as te:
                    if 'unexpected keyword argument' in str(te):
                        continue
                    raise
            return None

    def s16(self, value):
        val = int(value) & 0xFFFF
        return val - 65536 if val >= 32768 else val

    def constrain(self, val, min_val, max_val):
        return min(max_val, max(min_val, val))

    def connect(self):
        if self.gripper_rtu.is_socket_open():
            self.is_connect = True
            return True
        self.is_connect = bool(self.gripper_rtu.connect())
        return self.is_connect

    def set_rtu_mode(self):
        """
        Activate gripper matching Apicoo protocol:
          1. Release emergency stop & clear error: write reg 0 = 0
          2. Normal Mode (disable IO mode): write reg 257 = 0
             (0 = Normal Modbus Mode, 1 = IO Mode)
        """
        try:
            # Step 1: Release EMCY / Clear Errors (register 0 = 0)
            res0 = self._write_registers(0, [0])
            if res0 is None or res0.isError():
                res0 = self._write_register(0, 0)
            if res0 is not None and res0.isError():
                self.last_write_error = f'Release EMCY failed: {res0}'
                return False
            time.sleep(0.05)

            # Step 2: Normal Modbus Mode (register 257 = 0)
            res1 = self._write_registers(257, [0])
            if res1 is None or res1.isError():
                res1 = self._write_register(257, 0)
            if res1 is not None and res1.isError():
                self.last_write_error = f'Activate Normal mode failed: {res1}'
                return False
            time.sleep(0.1)

            self.last_write_error = None
            return True
        except Exception as exc:
            self.last_write_error = f'Activation failed: {exc}'
            return False

    def rtu_set_pos_pvt(self, pos, vel=100, tor=50):
        """
        Move gripper to target position.
        pos: Target opening in mm [0 .. 130]
        vel: Speed in mm/s [1 .. 150]
        tor: Force in % [1 .. 100]
        """
        pos = int(round(self.constrain(pos, 0, 130)))
        vel = int(round(self.constrain(vel, 1, 150)))
        tor = int(round(self.constrain(tor, 1, 100)))
        accel = 1000  # Default acceleration mm/s^2

        # Match Apicoo move protocol:
        # 1. Multi-register write [pos, vel, accel] (3 registers)
        # 2. Or single-register write [pos] (1 register)
        for payload in [[pos, vel, accel], [pos]]:
            try:
                res = self._write_registers(1, payload)
                if res is not None and not res.isError():
                    self.last_write_error = None
                    return True
            except Exception:
                pass

        # Fallback to single-register write (FC 06) for position
        try:
            res = self._write_register(1, pos)
            if res is not None and not res.isError():
                self.last_write_error = None
                return True
            self.last_write_error = f'Write position failed: {res}'
            return False
        except Exception as exc:
            self.last_write_error = f'Write position exception: {exc}'
            return False

    def rtu_write_pos(self, pos):
        self.rtu_set_pos_pvt(pos)

    def rtu_write_pos2(self, pos):
        self.rtu_set_pos_pvt(pos)

    def rtu_set_pos(self, pos):
        self.rtu_set_pos_pvt(pos)

    def rtu_set_vel(self, vel):
        pass

    def rtu_set_tor(self, tor):
        pass

    def reload_data(self):
        """
        Read telemetry from Input Registers starting at 0 (count=8):
          [0]: INFO (0x562f)
          [1]: POSITION (mm)
          [2]: SPEED (mm/s)
          [3]: RMS (force / current)
          [4]: ERROR
          [5]: MOTION (1=moving, 0=stopped)
          [6]: OBJ (1=detected, 0=none)
          [7]: EMCY
        """
        if not self.gripper_rtu.is_socket_open():
            self.last_read_error = 'serial connection is not open'
            return None
        try:
            registers = self._read_input_registers(0, count=8)
        except Exception as exc:
            self.last_read_error = f'Modbus read raised {type(exc).__name__}: {exc}'
            return None
        if registers is None:
            self.last_read_error = 'Modbus read returned no response'
            return None
        if registers.isError():
            self.last_read_error = f'Modbus error response: {registers}'
            return None
        if len(registers.registers) < 2:
            self.last_read_error = f'Modbus response contained only {len(registers.registers)} registers'
            return None

        self.last_read_error = None
        raw = registers.registers
        pos_mm = self.s16(raw[1])
        in_motion = raw[5] if len(raw) > 5 else 0
        obj_detected = raw[6] if len(raw) > 6 else 0
        err_code = raw[4] if len(raw) > 4 else 0
        raw_rms = self.s16(raw[3]) if len(raw) > 3 else 0

        # Chuẩn hóa format data tương thích hoàn toàn với hardware_interface.py:
        # data[0]: bit 7 là gMOVE (1 nếu đang chạy, 0 nếu dừng)
        #          bit 4..5 là OBJ (2 nếu close object)
        #          bit 8..15 là fault code
        status = 0
        if in_motion:
            status |= (1 << 7)  # bit 7: moving
        if obj_detected:
            status |= (2 << 4)  # bit 4..5: object grasped
        if err_code:
            status |= (err_code << 8)

        # data[2] = pos_mm * 2 (vì hardware_interface.py tính: float(data[2]) / 2.0)
        compat_data = [
            status,             # data[0]: status bitfield
            err_code,           # data[1]: error
            int(pos_mm * 2),    # data[2]: pos mm * 2
            raw_rms,            # data[3]: force / RMS (signed)
            0,                  # data[4]: temp
            in_motion,          # data[5]: motion
            obj_detected        # data[6]: obj
        ]
        return compat_data

    def check_movedone(self):
        data = self.reload_data()
        if data is not None:
            return 0 if (data[0] & 0x80) else 1
        return 1

    def check_object(self):
        data = self.reload_data()
        if data is not None:
            return "CLOSE" if (data[0] & (0x03 << 4)) else "NONE"
        return "NONE"

    def deactive(self):
        try:
            self._write_register(0, 65535)  # engage EMCY
        except Exception:
            pass

    def active(self):
        self.set_rtu_mode()

    def disconnect(self):
        self.gripper_rtu.close()
        self.is_connect = False
