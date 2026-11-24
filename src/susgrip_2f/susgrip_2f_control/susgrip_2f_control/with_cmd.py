import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')
        self._action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

    def send_goal(self, position, max_effort=50.0):
        # Hỗ trợ cả 2 cách nhập:
        # Nếu nhập position > 0.065 (ví dụ: 10, 20, 50, 100 mm), tự động chia cho 2000 để ra mét / ngón
        if position > 0.065:
            pos_m = (position / 2.0) / 1000.0
            self.get_logger().info(f'Đã nhận {position:.1f} mm -> Chuyển đổi thành {pos_m:.4f} m (mỗi ngón)')
        else:
            pos_m = position
            self.get_logger().info(f'Gửi mục tiêu: {pos_m:.4f} m ({pos_m * 2000.0:.1f} mm)')

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = pos_m
        goal_msg.command.max_effort = float(max_effort)

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action Server /gripper_controller/gripper_cmd không phản hồi!')
            return

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        total_mm = feedback.position * 2000.0
        self.get_logger().info(f'Feedback: vị trí={total_mm:.1f} mm ({feedback.position:.4f} m), effort={feedback.effort:.1f}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal bị từ chối!')
            return

        self.get_logger().info('Goal đã được chấp nhận, gripper đang di chuyển...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        total_mm = result.position * 2000.0
        self.get_logger().info(f'Hoàn thành! Vị trí đạt được: {total_mm:.1f} mm ({result.position:.4f} m)')

def main(args=None):
    rclpy.init(args=args)
    node = GripperClient()

    # Chạy vòng lặp spin trong một background thread để nhận feedback và callback liên tục
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("=" * 60)
    print("   TRÌNH ĐIỀU KHIỂN SUSGRIP 2F - DÒNG LỆNH (CLI)")
    print("   - Nhập theo mm: từ 0 đến 130 (VD: 0 = đóng, 130 = mở hết, 50 = 50mm)")
    print("   - Hoặc nhập theo mét: từ 0.0 đến 0.065 (VD: 0.065)")
    print("   - Gõ 'q' hoặc 'exit' để thoát")
    print("=" * 60)

    try:
        while rclpy.ok():
            try:
                user_input = input("\n👉 Nhập vị trí mong muốn: ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not user_input:
                continue
            if user_input.lower() in ['q', 'exit', 'quit']:
                break

            try:
                val = float(user_input)
                node.send_goal(val, 50.0)
            except ValueError:
                print("❌ Vui lòng nhập số hợp lệ!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()