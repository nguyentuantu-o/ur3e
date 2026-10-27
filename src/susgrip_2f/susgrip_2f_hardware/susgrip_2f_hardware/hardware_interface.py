#!/usr/bin/env python3
import sys,time
import os
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from control_msgs.action import GripperCommand
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Ensure SusGripLib can be imported
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from susgrip_2f_hardware.SusGripLib import SusGrip2F

class SusGrip2FHardwareInterface(Node):
    def __init__(self):
        super().__init__('susgrip_2f_hardware_interface')
        self.get_logger().info('SusGrip 2F Standalone Hardware Interface & Action Server starting...')
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('slave_id', 2)
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        slave_id = self.get_parameter('slave_id').get_parameter_value().integer_value
        self.is_connected = False
        self.state_read_failures = 0
        self._action_in_progress = False
        self.tf_broadcaster = TransformBroadcaster(self)

        try:
            self.sus_2f = SusGrip2F(port=serial_port, id=slave_id)
            if not self.sus_2f.connect():
                raise ConnectionError(
                    'pymodbus could not open the serial port. Check that no other process '
                    f'is using {serial_port}, and verify the USB/RS-485 adapter is connected.'
                )

            # Activate + set RTU mode (mirrors the Apicoo Universal Console init sequence).
            # The gripper boots in an unpowered / GPIO state; without the activate step
            # (reg 2000 = 1) first, the RTU-mode write (reg 2000 = 3) is silently ignored
            # and all subsequent Modbus reads fail — the same error seen when running
            # without the Apicoo console.
            self.get_logger().info(
                f'Activating gripper (slave ID {slave_id}) and switching to RTU mode …'
            )
            MAX_INIT_RETRIES = 3
            for attempt in range(1, MAX_INIT_RETRIES + 1):
                if self.sus_2f.set_rtu_mode():
                    break
                reason = getattr(self.sus_2f, 'last_write_error', None)
                self.get_logger().warn(
                    f'RTU-mode init attempt {attempt}/{MAX_INIT_RETRIES} failed: '
                    f'{reason or "no error detail"}. Retrying …'
                )
                time.sleep(0.5)
            else:
                reason = getattr(self.sus_2f, 'last_write_error', None)
                raise RuntimeError(
                    f'the serial port opened, but the gripper did not acknowledge the '
                    f'Modbus-RTU mode command after {MAX_INIT_RETRIES} attempts '
                    f'(slave ID {slave_id}): {reason or "invalid response"}'
                )

            initial_data = self.sus_2f.reload_data()
            if initial_data is None or len(initial_data) < 2:
                reason = getattr(self.sus_2f, 'last_read_error', None)
                raise RuntimeError(
                    'serial port opened, but no valid Modbus gripper state was '
                    f'read (slave ID {slave_id}): {reason or "invalid response"}'
                )
            self.get_logger().info(f"Connected to gripper successfully on {serial_port} (ID: {slave_id})!")
            self.is_connected = True
        except (FileNotFoundError, PermissionError, ConnectionError) as exc:
            self.get_logger().error(f"Failed to connect to gripper on {serial_port}: {exc}")
            self.is_connected = False
        except Exception as exc:
            self.get_logger().error(f"Failed to connect to gripper on {serial_port}: {exc}")
            self.get_logger().error(
                'The USB serial port opened but the SusGrip sent no Modbus response. '
                'This is a physical RS-485 link issue: verify gripper power, use a USB-to-RS485 '
                'adapter (not TTL UART), and verify the RS-485 A/B wiring to pins 3 and 8.'
            )
            self.is_connected = False

        # Multi-threaded execution group for concurrent action handling and telemetry
        self.callback_group = ReentrantCallbackGroup()
        # Dynamic parameter values for speed and force (default to 50%)
        self.current_vel = 80.0
        self.current_tor = 50.0

        # Subscriptions for speed and force (like the old driver)
        self.vel_sub = self.create_subscription(
            Float32,
            '/susgrip_2f/vel',
            self.vel_callback,
            10,
            callback_group=self.callback_group
        )
        self.force_sub = self.create_subscription(
            Float32,
            '/susgrip_2f/force',
            self.force_callback,
            10,
            callback_group=self.callback_group
        )

        # Publish directly to the topic consumed by MoveIt. This driver is
        # launched only for real-hardware runs, so an intermediate topic and
        # launch remap add no value and can leave MoveIt without gripper state
        # when separate source and remap configurations drift apart.
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.05, self.publish_joint_states, callback_group=self.callback_group)

        # Standard control_msgs GripperCommand Action Server
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            execute_callback=self.execute_action_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        if self.is_connected:
            self.get_logger().info('GripperCommand Action Server is active at /gripper_controller/gripper_cmd.')
        else:
            self.get_logger().warning(
                'GripperCommand Action Server is available, but it will reject goals until '
                'the Modbus connection succeeds.'
            )

    def vel_callback(self, msg):
        self.current_vel = max(1.0, min(100.0, msg.data))
        self.get_logger().info(f"Updated gripper velocity limit: {self.current_vel}%")

    def force_callback(self, msg):
        self.current_tor = max(1.0, min(100.0, msg.data))
        self.get_logger().info(f"Updated gripper force (torque) limit: {self.current_tor}%")

    def _publish_gripper_state(self, finger_pos, effort=0.0):
        # 1. Publish JointState for MoveIt CurrentStateMonitor
        now = self.get_clock().now()
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = ["gripper_joint", "gripper_joint_mimic"]
        msg.position = [float(finger_pos), float(finger_pos)]
        msg.velocity = [0.0, 0.0]
        msg.effort = [float(effort), float(effort)]
        self.joint_pub.publish(msg)

        # 2. Publish TF transforms for gripper links (parent: tool0)
        stamp = now.to_msg()
        transforms = []

        # tool0 -> spacer_cylinder
        t_spacer = TransformStamped()
        t_spacer.header.stamp = stamp
        t_spacer.header.frame_id = "tool0"
        t_spacer.child_frame_id = "spacer_cylinder"
        t_spacer.transform.translation.x = 0.0
        t_spacer.transform.translation.y = 0.0
        t_spacer.transform.translation.z = 0.0075
        t_spacer.transform.rotation.w = 1.0
        transforms.append(t_spacer)

        # tool0 -> gripper_body
        t_body = TransformStamped()
        t_body.header.stamp = stamp
        t_body.header.frame_id = "tool0"
        t_body.child_frame_id = "gripper_body"
        t_body.transform.translation.x = 0.0
        t_body.transform.translation.y = 0.0
        t_body.transform.translation.z = 0.015
        t_body.transform.rotation.w = 1.0
        transforms.append(t_body)

        # gripper_body -> gripper_tcp
        t_tcp = TransformStamped()
        t_tcp.header.stamp = stamp
        t_tcp.header.frame_id = "gripper_body"
        t_tcp.child_frame_id = "gripper_tcp"
        t_tcp.transform.translation.x = 0.0
        t_tcp.transform.translation.y = 0.0
        t_tcp.transform.translation.z = 0.152
        t_tcp.transform.rotation.w = 1.0
        transforms.append(t_tcp)

        # gripper_body -> finger_left (prismatic: origin [0.0135, 0, 0.112], yaw=pi, axis=[-1,0,0])
        # Joint displacement along -x in child frame = +x in gripper_body frame
        t_left = TransformStamped()
        t_left.header.stamp = stamp
        t_left.header.frame_id = "gripper_body"
        t_left.child_frame_id = "finger_left"
        t_left.transform.translation.x = 0.0135 + float(finger_pos)
        t_left.transform.translation.y = 0.0
        t_left.transform.translation.z = 0.112
        t_left.transform.rotation.x = 0.0
        t_left.transform.rotation.y = 0.0
        t_left.transform.rotation.z = 1.0
        t_left.transform.rotation.w = 0.0
        transforms.append(t_left)

        # gripper_body -> finger_right (prismatic mimic: origin [-0.0135, 0, 0.112], yaw=0, axis=[-1,0,0])
        t_right = TransformStamped()
        t_right.header.stamp = stamp
        t_right.header.frame_id = "gripper_body"
        t_right.child_frame_id = "finger_right"
        t_right.transform.translation.x = -0.0135 - float(finger_pos)
        t_right.transform.translation.y = 0.0
        t_right.transform.translation.z = 0.112
        t_right.transform.rotation.x = 0.0
        t_right.transform.rotation.y = 0.0
        t_right.transform.rotation.z = 0.0
        t_right.transform.rotation.w = 1.0
        transforms.append(t_right)

        self.tf_broadcaster.sendTransform(transforms)

    def publish_joint_states(self):
        if not self.is_connected or self._action_in_progress:
            return
        try:
            data = self.sus_2f.reload_data()
        except Exception as exc:
            data = None
            error = str(exc)
        else:
            error = getattr(self.sus_2f, 'last_read_error', None)
        if data is None or not isinstance(data, list) or len(data) < 2:
            self.state_read_failures += 1
            if self.state_read_failures == 1 or self.state_read_failures % 100 == 0:
                self.get_logger().error(
                    'Cannot publish gripper joint state: Modbus read returned '
                    f'no valid position (failure {self.state_read_failures}): '
                    f'{error or "invalid register response"}. MoveIt will keep '
                    'the robot state incomplete until telemetry recovers.'
                )
            return
        if self.state_read_failures:
            self.get_logger().info(
                f'Gripper telemetry recovered after {self.state_read_failures} '
                'failed reads.'
            )
            self.state_read_failures = 0

        current_mm = float(data[2]) / 2.0
        finger_pos = (current_mm / 2.0) / 1000.0
        effort = float(data[3]) / 10.0 if len(data) > 3 else 0.0
        self._publish_gripper_state(finger_pos, effort)

    # --- Action Goal Handling ---
    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received new gripper goal request: {goal_request.command.position} (meters)')
        if not self.is_connected:
            self.get_logger().error('Rejecting gripper goal: physical Modbus connection is unavailable.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel active gripper goal')
        return CancelResponse.ACCEPT

    def execute_action_callback(self, goal_handle):
        self.get_logger().info('Executing gripper goal...')

        if not self.is_connected:
            self.get_logger().error(
                'Rejecting gripper motion: physical gripper is not connected.'
            )
            result = GripperCommand.Result()
            result.reached_goal = False
            goal_handle.abort()
            return result
        
        self._action_in_progress = True
        try:
            # The action goal position represents a single finger's position in meters.
            # Total physical opening is 2x this amount, converted to mm.
            target_mm = round(goal_handle.request.command.position * 2000.0)

            # Override target torque if specified via action max_effort (if non-zero)
            target_effort = self.current_tor

            if goal_handle.request.command.max_effort > 0.0:
                target_effort = max(1.0, min(100.0, goal_handle.request.command.max_effort))

            # Guard physical boundaries
            target_mm = max(0.0, min(130.0, target_mm))
            self.get_logger().info(f'Action Target Physical opening: {target_mm:.1f} mm | Speed: {self.current_vel}% | Force: {target_effort}%')

            if self.is_connected:
                # Check for faults and auto-reset if necessary
                data = self.sus_2f.reload_data()
                if data is not None and len(data) > 0:
                    status = data[0]
                    fault_code = status >> 8
                    if fault_code != 0:
                        self.get_logger().warn(f"Gripper is in FAULT state (Code: {fault_code}). Auto-resetting before move...")
                        self.sus_2f.set_rtu_mode()
                        time.sleep(0.1)

                ok = self.sus_2f.rtu_set_pos_pvt(int(target_mm), int(self.current_vel), int(target_effort))
                if not ok:
                    self.get_logger().error(f"Failed to send motion command to gripper: {getattr(self.sus_2f, 'last_write_error', 'unknown error')}")
                else:
                    self.get_logger().info(f"Motion command sent successfully to gripper: target={target_mm} mm")
                time.sleep(0.2)

            # Loop and monitor feedback until movement completes
            feedback_msg = GripperCommand.Feedback()
            feedback_msg.reached_goal = False
            result = GripperCommand.Result()
            rate = self.create_rate(20) # 20Hz monitor loop
            timeout = 30.0

            start_time = self.get_clock().now()
            last_pos = -999.0
            last_pos_time = 0.0

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Gripper action goal canceled.')
                    result.reached_goal = False
                    return result

                # Read current position for feedback
                if self.is_connected:
                    data = self.sus_2f.reload_data()
                    if data is not None and len(data) >= 6:
                        current_mm = float(data[2]) / 2.0
                        finger_pos = (current_mm / 2.0) / 1000.0
                        effort = float(data[3]) / 10.0

                        # Continuously publish /joint_states and TF frames so RViz & MoveIt animate in real time
                        self._publish_gripper_state(finger_pos, effort)

                        # Register 1000 (data[0]) bit 7 is gMOVE: 1 while moving.
                        is_stopped = 1 if (int(data[0]) & 0x80) == 0 else 0
                        
                        # Timeout check and Elapsed time
                        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9

                        # Stall detection: if position barely changes over 1.5 seconds, force stop!
                        if abs(current_mm - last_pos) > 0.5:
                            last_pos = current_mm
                            last_pos_time = elapsed
                        elif elapsed - last_pos_time > 1.5:
                            is_stopped = 1  # Force stopped status

                        # report feedback in meters for a single finger
                        feedback_msg.position = finger_pos
                        feedback_msg.effort = effort
                        feedback_msg.reached_goal = False
                        goal_handle.publish_feedback(feedback_msg)

                        if int(elapsed * 10) % 10 == 0:
                            self.get_logger().info(f"Monitor - Pos: {current_mm:.1f}mm ({finger_pos:.4f}m),\n"
                                                   f"Target: {target_mm:.1f}mm,\n"
                                                   f"Stopped: {is_stopped}")

                        # Check completion:
                        is_opening = (target_mm > current_mm + 5.0)
                        reached_target = (abs(current_mm - target_mm) <= 3.0)

                        if reached_target:
                            self.get_logger().info(f'Target reached successfully! Current: {current_mm:.1f} mm')
                            result.reached_goal = True
                            feedback_msg.reached_goal = True
                            goal_handle.publish_feedback(feedback_msg)
                            goal_handle.succeed()
                            break
                        elif is_stopped == 1 and elapsed > 1.2:
                            if not is_opening:
                                # Closing motion stopped before target -> object grasped!
                                self.get_logger().info(f'Gripper stopped early (object grasped). Current: {current_mm:.1f} mm')
                                result.reached_goal = True
                                feedback_msg.reached_goal = True
                                goal_handle.publish_feedback(feedback_msg)
                                goal_handle.succeed()
                                break
                            elif elapsed > 3.0:
                                # Opening motion stopped without reaching target after sufficient time
                                self.get_logger().warn(f'Opening motion completed or stalled at {current_mm:.1f} mm (target {target_mm:.1f} mm)')
                                result.reached_goal = True
                                feedback_msg.reached_goal = True
                                goal_handle.publish_feedback(feedback_msg)
                                goal_handle.succeed()
                                break
                        elif elapsed > timeout:
                            self.get_logger().warn('Gripper move execution timed out!')
                            result.reached_goal = False
                            feedback_msg.reached_goal = False
                            goal_handle.publish_feedback(feedback_msg)
                            goal_handle.abort()
                            break
                rate.sleep()

            # Populate final result in meters per finger
            if self.is_connected:
                data = self.sus_2f.reload_data()
                if data is not None and len(data) >= 3:
                    current_mm = float(data[2]) / 2.0
                    result.position = (current_mm / 2.0) / 1000.0
                    effort = float(data[3]) / 10.0 if len(data) > 3 else 0.0
                    self._publish_gripper_state(result.position, effort)
            result.stalled = False
            return result
        finally:
            self._action_in_progress = False

    def close_serial(self):
        """Close serial connection when shutting down"""
        if self.is_connected:
            self.sus_2f.disconnect()
            self.get_logger().info("Closed gripper serial connection.")


def main(args=None):
    rclpy.init(args=args)
    node = SusGrip2FHardwareInterface()
    
    # Use multi-threaded executor to allow feedback updates while goal runs
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down SusGrip 2F Hardware Node...")
        node.close_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
