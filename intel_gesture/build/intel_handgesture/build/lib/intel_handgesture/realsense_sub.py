import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rcl_interfaces.srv import GetParameters 

class IntelSubscriber(Node):
    def __init__(self):
        super().__init__("intel_subscriber")
        self.intrinsics = None
        self.intrinsics_ready = False

        # Create subscriptions 
        self.subscription_rgb = self.create_subscription(
            Image, "rgb_frame", self.rgb_frame_callback, 10)
        self.subscription_depth = self.create_subscription(
            Image, "depth_frame", self.depth_frame_callback, 10)
        self.rgb_frame = None
        self.depth_frame = None
        self.br_rgb = CvBridge()
        self.br_depth = CvBridge()
        self.rgb_frame = None  

        # Create the parameter client
        self.parameter_client = self.create_client(GetParameters, '/intel_publisher/get_parameters')

        # Start the parameter check timer
        self.create_timer(1.0, self.check_parameters_timer_callback)

    def check_parameters_timer_callback(self):
        if not self.intrinsics_ready:
            self.send_parameter_request()

    def send_parameter_request(self):
        # Wait for the service to be available
        while not self.parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Parameter service not available, waiting again...')

        req = GetParameters.Request()
        req.names = ['camera_intrinsics']
        future = self.parameter_client.call_async(req)
        future.add_done_callback(self.parameter_response_callback)

    def parameter_response_callback(self, future):
        try:
            response = future.result()
            self.intrinsics = response.values[0].double_array_value 
            if len(self.intrinsics) == 9:
                fx, fy, ppx, ppy = self.intrinsics[0], self.intrinsics[4], self.intrinsics[2], self.intrinsics[5]
                self.get_logger().info(f"Camera Intrinsics: fx: {fx}, fy: {fy}, ppx: {ppx}, ppy: {ppy}")
                self.intrinsics_ready = True
            else:
                self.get_logger().warn("Received camera intrinsics are not complete.")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

    def rgb_frame_callback(self, data):
        self.get_logger().info("Receiving RGB frame")
        self.rgb_frame = self.br_rgb.imgmsg_to_cv2(data, "bgr8")

    def depth_frame_callback(self, data):
        self.get_logger().info("Receiving Depth frame")
        self.depth_frame = self.br_depth.imgmsg_to_cv2(data, desired_encoding='passthrough')

    def get_latest_rgb(self):
        return self.rgb_frame

    def get_latest_depth(self):
        return self.depth_frame

def main(args=None):
    rclpy.init(args=args)
    intel_subscriber = IntelSubscriber()
    rclpy.spin(intel_subscriber)
    intel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()