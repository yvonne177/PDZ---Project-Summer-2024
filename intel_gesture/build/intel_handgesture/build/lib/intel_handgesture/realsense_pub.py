import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2
import json

class IntelPublisher(Node):
    def __init__(self):
        super().__init__("intel_publisher")
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frame", 10)
        self.intel_publisher_depth = self.create_publisher(Image, "depth_frame", 10)
        self.br_rgb = CvBridge()
        self.rgb_image = None

        # Load JSON configuration
        self.load_json_config("realsense_config.json")
        self.filters = []  # Initialize filters list

        self.initialize_realsense()
        self.create_timer(0.0333333, self.timer_callback)

    def load_json_config(self, json_file):
        try:
            with open(json_file, 'r') as f:
                self.config_data = json.load(f)
            self.json_string = str(self.config_data).replace("'", '"')
        except FileNotFoundError:
            self.get_logger().error(f"JSON configuration file not found: {json_file}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Error decoding JSON configuration file: {json_file}")

    def initialize_realsense(self):
        try:
            self.pipe = rs.pipeline()
            self.cfg = rs.config()

            # Enable color and depth streams with custom resolution
            self.cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
            self.cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
            self.profile = self.pipe.start(self.cfg)

            # Retrieve and store intrinsics once
            frames = self.pipe.wait_for_frames()
            color_frame = frames.get_color_frame()
            self.intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

            # Extract and set intrinsic parameters as ROS2 parameters
            self.set_camera_intrinsics()

            # Apply Advanced Mode Configuration
            dev = self.profile.get_device()
            advnc_mode = rs.rs400_advanced_mode(dev)
            advnc_mode.load_json(self.json_string)
            self.get_logger().info("Loaded JSON configuration to RealSense camera") 

            # Apply post-processing filters
            self.filters = apply_post_processing_filters(self.config_data)

        except Exception as e:
            self.get_logger().error("Intel RealSense is not connected: " + str(e))

    def set_camera_intrinsics(self):
        fx = self.intrinsics.fx
        fy = self.intrinsics.fy
        ppx = self.intrinsics.ppx
        ppy = self.intrinsics.ppy

        self.declare_parameter('camera_intrinsics', [0.0] * 9)
        intrinsic_list = [
            fx, 0.0, ppx,
            0.0, fy, ppy,
            0.0, 0.0, 1.0
        ]
        self.set_parameters([
            Parameter(
                'camera_intrinsics',
                Parameter.Type.DOUBLE_ARRAY,
                intrinsic_list
            )
        ])
        self.get_logger().info("Set camera_intrinsics parameter")
        self.get_logger().info(f"Formatted Camera Intrinsics: {intrinsic_list}")

    def timer_callback(self):
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if color_frame and depth_frame:
            # Convert frames to numpy arrays
            self.rgb_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Apply post-processing filters to depth frame
            for filter in self.filters:
                depth_frame = filter.process(depth_frame)
            
            # Convert depth image to 8-bit for visualization
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_image_8u = cv2.convertScaleAbs(depth_image, alpha=0.0255)

            # Publish images
            self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(self.rgb_image, "bgr8"))
            self.intel_publisher_depth.publish(self.br_rgb.cv2_to_imgmsg(depth_image_8u, "mono8"))
            self.get_logger().info("Publishing RGB and Depth frames")

# Function to apply post-processing filters
def apply_post_processing_filters(config):
    filters = []

    # Apply Decimation Filter
    if config.get("post-processing-decimation-filter") == "True":
        decimation_filter = rs.decimation_filter()
        filters.append(decimation_filter)

    # Apply HDR Merge Filter
    if config.get("post-processing-hdr-merge") == "True":
        hdr_merge_filter = rs.hdr_merge()
        filters.append(hdr_merge_filter)

    # Apply Threshold Filter
    threshold_config = config.get("post-processing-threshold-filter")
    if threshold_config:
        threshold_filter = rs.threshold_filter()
        threshold_filter.set_option(rs.option.min_distance, float(threshold_config.get("min_distance", 0.1)))
        threshold_filter.set_option(rs.option.max_distance, float(threshold_config.get("max_distance", 1.7)))
        filters.append(threshold_filter)

    # Apply Depth to Disparity Filter
    if config.get("post-processing-depth-to-disparity") == "True":
        depth_to_disparity_filter = rs.disparity_transform(True)
        filters.append(depth_to_disparity_filter)

    # Apply Spatial Filter
    spatial_config = config.get("post-processing-spatial-filter")
    if spatial_config:
        spatial_filter = rs.spatial_filter()
        spatial_filter.set_option(rs.option.filter_smooth_alpha, float(spatial_config.get("smooth_alpha", 0.92)))
        spatial_filter.set_option(rs.option.filter_smooth_delta, float(spatial_config.get("smooth_delta", 20)))
        spatial_filter.set_option(rs.option.filter_magnitude, float(spatial_config.get("magnitude", 4)))
        spatial_filter.set_option(rs.option.filter_hole_filling, int(spatial_config.get("hole_filling", 0)))
        filters.append(spatial_filter)

    # Apply Temporal Filter
    temporal_config = config.get("post-processing-temporal-filter")
    if temporal_config:
        temporal_filter = rs.temporal_filter()
        temporal_filter.set_option(rs.option.filter_smooth_alpha, float(temporal_config.get("filter_smooth_alpha", 0.63)))
        temporal_filter.set_option(rs.option.filter_smooth_delta, float(temporal_config.get("filter_smooth_delta", 66)))
        filters.append(temporal_filter)

    # Apply Disparity to Depth Filter
    if config.get("post-processing-disparity-to-depth") == "True":
        disparity_to_depth_filter = rs.disparity_transform(False)
        filters.append(disparity_to_depth_filter)

    return filters

def main(args=None):
    rclpy.init(args=args)
    intel_publisher = IntelPublisher()
    rclpy.spin(intel_publisher)
    intel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()