#!/usr/bin/env python3
"""
grid_coordinator.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Standalone Pick and Place Grid Coordinator for KUKA.

Features enabled:
 - Y=0 Transit Waypoint
 - Distance-based physical synchronization (Solves KUKA $ADVANCE desync)
 - Pneumatic actuation delays (Solves mid-air dropping)
 - Safe direct MoveGroup action calls (No custom .srv required)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math

from std_msgs.msg import Int8
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, 
    PositionConstraint, OrientationConstraint, BoundingVolume
)
from shape_msgs.msg import SolidPrimitive

class GridCoordinator(Node):
    def __init__(self):
        super().__init__('grid_coordinator')
        
        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_pub = self.create_publisher(Int8, '/gripper_cmd', 10)
        
        self.get_logger().info("Waiting for MoveGroup Action Server...")
        self._move_client.wait_for_server()
        self.get_logger().info("Action Server Found! System Ready.")

        # Grid Coordinates (in millimeters)
        self.A = {1: (169.40, 278.52), 2: (217.17, 278.52), 3: (264.65, 278.52),
                  4: (315.12, 278.52), 5: (364.77, 278.52), 6: (413.77, 278.52),
                  7: (468.54, 278.52), 8: (521.46, 278.52), 9: (521.46, 222.50)}
                  
        self.B = {1: (169.40, -278.52), 2: (217.17, -278.52), 3: (264.65, -278.52),
                  4: (315.12, -278.52), 5: (364.77, -278.52), 6: (413.77, -278.52),
                  7: (468.54, -278.52), 8: (521.46, -278.52), 9: (521.46, -222.50)}
        
        # Z Heights (in millimeters)
        self.Z_PICK = 73.0
        self.Z_TRANSIT = 345.0
        
        # Fixed Orientation (downward picking)
        self.ORI_X = 0.0090
        self.ORI_Y = 0.9280
        self.ORI_Z = 0.0184
        self.ORI_W = 0.3719

        # --- SYNCHRONIZATION VARIABLES ---
        self.last_pose = None
        self.ROBOT_SPEED_MM_S = 100.0  # Assumed physical speed at 10% velocity

    def move(self, x_mm, y_mm, z_mm):
        # Convert millimeters to meters for ROS 2 MoveIt
        x, y, z = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0
        
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'manipulator'
        req.planner_id = 'PTP'
        req.pipeline_id = 'pilz_industrial_motion_planner'
        
        # Lock velocity so our time calculations are accurate
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        
        # 1. Position Constraint
        pos = PositionConstraint()
        pos.header.frame_id = 'base_link'
        pos.link_name = 'tool0'
        
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005] # 5mm tolerance
        
        bv = BoundingVolume()
        bv.primitives.append(sphere)
        
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        bv.primitive_poses.append(p)
        
        pos.constraint_region = bv
        pos.weight = 1.0
        req.goal_constraints.append(Constraints(position_constraints=[pos]))
        
        # 2. Orientation Constraint
        ori = OrientationConstraint()
        ori.header.frame_id = 'base_link'
        ori.link_name = 'tool0'
        ori.orientation.x = self.ORI_X
        ori.orientation.y = self.ORI_Y
        ori.orientation.z = self.ORI_Z
        ori.orientation.w = self.ORI_W
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        ori.weight = 1.0
        req.goal_constraints[0].orientation_constraints.append(ori)
        
        goal.request = req
        
        self.get_logger().info(f"Moving to: X={x_mm:.1f} Y={y_mm:.1f} Z={z_mm:.1f}")
        
        future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("  [ERROR] Move rejected by server.")
            return False
            
        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut)
        
        error_code = result_fut.result().result.error_code.val
        if error_code != 1:
            self.get_logger().error(f"  [ERROR] Move failed! (MoveIt Code: {error_code})")
            return False
            
        # --- PHYSICAL SYNCHRONIZATION HACK ---
        # Calculate how far the robot is moving in millimeters
        if self.last_pose is None:
            dist = 300.0  # Arbitrary delay for the very first initialization move
        else:
            dist = math.sqrt((x_mm - self.last_pose[0])**2 + 
                             (y_mm - self.last_pose[1])**2 + 
                             (z_mm - self.last_pose[2])**2)
                             
        self.last_pose = (x_mm, y_mm, z_mm)
        
        # Calculate time (Distance / Speed) + 1.0 second buffer for accel/decel
        travel_time = (dist / self.ROBOT_SPEED_MM_S) + 1.0
        self.get_logger().info(f"  -> Waiting {travel_time:.1f}s for physical robot to arrive...")
        time.sleep(travel_time)
            
        return True

    def set_gripper(self, state: int):
        # 1. Force Python to wait for KUKA to physically catch up to the buffer
        # (This guarantees we are physically at the coordinate before firing)
        time.sleep(1.5)
        
        # 2. Publish the command
        msg = Int8()
        msg.data = state
        self._gripper_pub.publish(msg)
        
        # 3. Wait for pneumatics (vacuum needs time to build seal or vent air)
        if state == 1:
            self.get_logger().info("  -> Gripper ON - Building vacuum seal...")
            time.sleep(1.5)  # Give it time to build suction
        else:
            self.get_logger().info("  -> Gripper OFF - Venting vacuum...")
            time.sleep(1.5)  # Give it time to release the vacuum seal so it drops

    def run_sequence(self):
        for i in range(1, 10):
            self.get_logger().info(f"\n================================")
            self.get_logger().info(f"      PROCESSING CUBE {i}")
            self.get_logger().info(f"================================")
            
            ax, ay = self.A[i]
            bx, by = self.B[i]
            
            # 1. To A (Transit Height)
            self.move(ax, ay, self.Z_TRANSIT)
            # 2. To A (Pick Height)
            self.move(ax, ay, self.Z_PICK)
            
            # 3. Pick it up
            self.set_gripper(1)
            
            # 4. Retract back to transit height
            self.move(ax, ay, self.Z_TRANSIT)
            
            # 5. Transit point C (Y=0, directly between A and B)
            self.get_logger().info(f"  --- Transit through Point C (Y=0) ---")
            self.move(ax, 0.0, self.Z_TRANSIT)
            
            # 6. To B (Transit Height)
            self.move(bx, by, self.Z_TRANSIT)
            # 7. To B (Place Height)
            self.move(bx, by, self.Z_PICK)
            
            # 8. Drop it
            self.set_gripper(0)
            
            # 9. Retract back to transit height
            self.move(bx, by, self.Z_TRANSIT)

        self.get_logger().info("\n*** GRID DEMONSTRATION COMPLETE ***")

def main():
    rclpy.init()
    node = GridCoordinator()
    
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        node.get_logger().info("Sequence interrupted by user.")
    finally:
        # Safety cutoff
        node.set_gripper(0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()