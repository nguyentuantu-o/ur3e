#!/usr/bin/env python3
"""
Test kết nối và đóng/mở SusGrip theo protocol mới.
Chạy: python3 test_move.py
"""

import time
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from SusGripLib import SusGrip2F

PORT = "/dev/ttyUSB0"
SLAVE_ID = 2

def main():
    print(f"👉 Khởi tạo kết nối tới Gripper trên {PORT} (ID={SLAVE_ID})...")
    grip = SusGrip2F(port=PORT, id=SLAVE_ID)
    
    if not grip.connect():
        print("❌ Không mở được cổng serial! Hãy kiểm tra cáp cắm hoặc quyền (sudo chmod 666 /dev/ttyUSB0).")
        return

    print("👉 Kích hoạt Gripper (set_rtu_mode)...")
    if not grip.set_rtu_mode():
        print(f"❌ Kích hoạt thất bại: {grip.last_write_error}")
        return
    print("✅ Kích hoạt THÀNH CÔNG!")

    print("\n👉 Đọc dữ liệu trạng thái ban đầu:")
    data = grip.reload_data()
    if data:
        current_pos = data[2] / 2.0
        print(f"   Vị trí hiện tại: {current_pos:.1f} mm")
        print(f"   Lực / RMS:      {data[3]}")
        print(f"   Mã lỗi:         {data[1]}")
    else:
        print(f"❌ Đọc telemetry thất bại: {grip.last_read_error}")
        return

    # Thử điều khiển đóng kẹp về 20mm
    print("\n👉 Thử lệnh điều khiển: Đóng kẹp về 20 mm...")
    ok = grip.rtu_set_pos_pvt(pos=20, vel=80, tor=50)
    if not ok:
        print(f"❌ Gửi lệnh đóng kẹp thất bại: {grip.last_write_error}")
    else:
        print("   Đang di chuyển...")
        time.sleep(2.0)
        data = grip.reload_data()
        if data:
            print(f"   Vị trí sau khi đóng: {data[2] / 2.0:.1f} mm")

    # Thử mở kẹp về 100mm
    print("\n👉 Thử lệnh điều khiển: Mở kẹp về 100 mm...")
    ok = grip.rtu_set_pos_pvt(pos=100, vel=80, tor=50)
    if not ok:
        print(f"❌ Gửi lệnh mở kẹp thất bại: {grip.last_write_error}")
    else:
        print("   Đang di chuyển...")
        time.sleep(2.0)
        data = grip.reload_data()
        if data:
            print(f"   Vị trí sau khi mở: {data[2] / 2.0:.1f} mm")

    grip.disconnect()
    print("\n🎉 HOÀN TẤT! Gripper đã phản hồi và hoạt động bình thường!")

if __name__ == "__main__":
    main()
