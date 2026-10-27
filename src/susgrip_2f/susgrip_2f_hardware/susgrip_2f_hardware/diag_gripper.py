#!/usr/bin/env python3
"""
Gripper Modbus Diagnostic Script v2
Thử đọc nhiều địa chỉ register + nhiều function code khác nhau.
Chạy: python3 diag_gripper.py
"""

import time, sys
from pymodbus.client import ModbusSerialClient

PORT      = "/dev/ttyUSB0"
SLAVE_ID  = 2
BAUDRATE  = 115200

# Địa chỉ register cần thử đọc (cả FC=03 và FC=04)
READ_ADDRS = [
    0,    # 0x0000
    1,    # 0x0001
    256,  # 0x0100
    257,  # 0x0101  ← Apicoo viết tới đây
    258,  # 0x0102
    999,  # 0x03E7
    1000, # 0x03E8  ← SusGripLib hiện tại
    1001, # 0x03E9
    2000, # 0x07D0  ← SusGripLib write
]

# Địa chỉ write cần thử
WRITE_TESTS = [
    (257,  1),   # reg 0x0101 = 1  ← Apicoo viết tới 0x0101
    (257,  3),   # reg 0x0101 = 3
    (257,  0),   # reg 0x0101 = 0
    (2000, 1),   # reg 2000 = 1  (activate trong SusGripLib)
    (2000, 3),   # reg 2000 = 3  (RTU mode trong SusGripLib)
]

def open_client():
    c = ModbusSerialClient(
        port=PORT, baudrate=BAUDRATE,
        parity='N', stopbits=1, bytesize=8,
        handle_local_echo=False,
        timeout=2.0, retries=1,
    )
    ok = c.connect()
    return c if ok else None

def main():
    import os
    if not os.path.exists(PORT):
        print(f"❌ Port {PORT} không tồn tại!")
        return

    print(f"=== Diag gripper slave_id={SLAVE_ID} baud={BAUDRATE} port={PORT} ===\n")

    # ── PHASE 1: thử đọc trực tiếp (không kích hoạt trước) ──────────────
    print("── Phase 1: Đọc trực tiếp ──")
    c = open_client()
    if not c:
        print("❌ Không mở được port!"); return

    for addr in READ_ADDRS:
        for fc, fname in [(4, "FC04-InputReg"), (3, "FC03-HoldingReg")]:
            try:
                if fc == 4:
                    r = c.read_input_registers(addr, count=2, device_id=SLAVE_ID)
                else:
                    r = c.read_holding_registers(addr, count=2, device_id=SLAVE_ID)

                if r is not None and not r.isError():
                    print(f"  ✅ {fname} addr={addr:5d} (0x{addr:04X}) → {[hex(v) for v in r.registers]}")
                elif r is not None and r.isError():
                    print(f"  ⚠️  {fname} addr={addr:5d} (0x{addr:04X}) → ERROR response: {r}")
                else:
                    print(f"  ❌ {fname} addr={addr:5d} (0x{addr:04X}) → Timeout/None")
            except Exception as e:
                print(f"  ❌ {fname} addr={addr:5d} (0x{addr:04X}) → Exception: {e}")
    c.close()

    # ── PHASE 2: gửi write rồi đọc ──────────────────────────────────────
    print("\n── Phase 2: Viết kích hoạt rồi đọc ──")
    for wr_addr, wr_val in WRITE_TESTS:
        c = open_client()
        if not c:
            print("❌ Không mở được port!"); continue

        try:
            wr = c.write_register(wr_addr, wr_val, device_id=SLAVE_ID)
            if wr is None:
                print(f"  WRITE reg{wr_addr}={wr_val} → timeout"); c.close(); continue
            if wr.isError():
                print(f"  WRITE reg{wr_addr}={wr_val} → ERROR: {wr}"); c.close(); continue
            print(f"  ✅ WRITE reg{wr_addr}={wr_val} OK → đợi 600ms rồi đọc...")
        except Exception as e:
            print(f"  WRITE reg{wr_addr}={wr_val} → Exception: {e}"); c.close(); continue

        time.sleep(0.6)

        for addr in [0, 257, 1000, 2000]:
            try:
                r = c.read_input_registers(addr, count=4, device_id=SLAVE_ID)
                if r is not None and not r.isError():
                    print(f"    ✅ FC04 addr={addr} → {[hex(v) for v in r.registers]}")
                else:
                    r2 = c.read_holding_registers(addr, count=4, device_id=SLAVE_ID)
                    if r2 is not None and not r2.isError():
                        print(f"    ✅ FC03 addr={addr} → {[hex(v) for v in r2.registers]}")
                    else:
                        print(f"    ❌ addr={addr} → no response")
            except Exception as e:
                print(f"    ❌ addr={addr} → {e}")
        c.close()
        time.sleep(0.3)

    print("\n=== Xong ===")

if __name__ == "__main__":
    main()
