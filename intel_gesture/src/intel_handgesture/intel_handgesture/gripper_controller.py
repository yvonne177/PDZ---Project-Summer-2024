import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from franka_msgs.action import Grasp, Move
from rclpy.action import ActionClient

class FrankaGripperControllerNode(Node):

    def __init__(self):
        super().__init__('franka_gripper_controller')
        
        self.subscription = self.create_subscription(
            String,
            'recognized_gesture',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Action clients for controlling the gripper
        self.gripper_move_client = ActionClient(self, Move, '/franka_gripper/move')
        self.gripper_grasp_client = ActionClient(self, Grasp, '/franka_gripper/grasp')

        self.get_logger().info("Franka Gripper Controller Node has been started.")

    def listener_callback(self, msg):
        gesture = msg.data
        self.get_logger().info(f"Received gesture: {gesture}")
        
        if gesture == 'Open Hand':
            self.open_gripper()
        elif gesture == 'Close Hand':
            self.close_gripper()
        else:
            self.get_logger().warn(f"Unrecognized gesture: {gesture}")

    def open_gripper(self):
        goal_msg = Move.Goal()
        goal_msg.width = 0.08  # Open to 8 cm
        goal_msg.speed = 0.1  # Speed of opening

        self.gripper_move_client.wait_for_server()
        self.gripper_move_client.send_goal_async(goal_msg)
        self.get_logger().info("Gripper opening...")

    def close_gripper(self):
        goal_msg = Grasp.Goal()
        goal_msg.width = 0.0  # Close gripper completely
        goal_msg.speed = 0.1  # Speed of closing
        goal_msg.force = 40.0  # Grasping force in N
        goal_msg.epsilon.inner = 0.005
        goal_msg.epsilon.outer = 0.005

        self.gripper_grasp_client.wait_for_server()
        self.gripper_grasp_client.send_goal_async(goal_msg)
        self.get_logger().info("Gripper closing...")

def main(args=None):
    rclpy.init(args=args)
    gripper_controller_node = FrankaGripperControllerNode()
    
    try:
        rclpy.spin(gripper_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        gripper_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
