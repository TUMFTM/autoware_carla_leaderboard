#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a ROS autonomous agent interface to control the ego vehicle via a ROS2 stack
"""

# Long-Term you need to get this outside in a seperate ros package

from __future__ import print_function
import time
import threading
import queue
import numpy as np
import math
import re

import carla

from leaderboard.utils.route_manipulation import downsample_route
from leaderboard.autoagents.autonomous_agent import Track
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.task import Future

from ackermann_msgs.msg import AckermannDrive
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, TransformStamped, AccelWithCovarianceStamped, PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2, Image
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import MarkerArray

from autoware_agent.tum_ros_base_agent import TUMROSBaseAgent, BridgeHelper
from autoware_agent.aw_converter import AutowareConverter
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState, RouteState, OperationModeState
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport, Engage
from autoware_control_msgs.msg import Control
from autoware_perception_msgs.msg import PredictedObjects, TrafficLightGroupArray, TrafficLightGroup, TrafficLightElement
from tier4_vehicle_msgs.msg import ActuationStatusStamped
from autoware_adapi_v1_msgs.srv import SetRoutePoints

def get_entry_point():
    return 'AutowarePriviligedAgent'

EPSILON = 0.001

def wait_for_message(node, topic, topic_type, timeout=None):

    s = None
    try:
        future = Future()
        s = node.create_subscription(
            topic_type,
            topic,
            lambda msg: future.set_result(msg.data),
            qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        rclpy.spin_until_future_complete(node, future, timeout)

    finally:
        if s is not None:
            node.destroy_subscription(s)

    return future.result()


class AutowarePriviligedAgent(TUMROSBaseAgent):


    ROS_VERSION = 2

    def __init__(self, carla_host, carla_port, debug=False):
        super(AutowarePriviligedAgent, self).__init__(self.ROS_VERSION, carla_host, carla_port, debug)
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node("autoware_priviliged_node")

        # --Beginning of Autoware Stuff--
        # Lidar Publisher using best effort
        self._localization_acceleration_subscriber = self.ros_node.create_subscription(AccelWithCovarianceStamped, "/localization/acceleration", self._acc_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._lidar_pub = self.ros_node.create_publisher(PointCloud2, "/sensor/lidar/front", qos_profile=qos_profile_sensor_data)
        # Subscriber for Autoware Priviliged
        self._clock_pub = self.ros_node.create_publisher(Clock, "/clock", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Publisher for Localization
        self._localization_publisher = self.ros_node.create_publisher(Odometry, "/localization/kinematic_state", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._tf_publisher = self.ros_node.create_publisher(TFMessage, "/tf", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._localization_init_state_publisher = self.ros_node.create_publisher(LocalizationInitializationState, "/localization/initialization_state", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._localization_acceleration_publisher = self.ros_node.create_publisher(AccelWithCovarianceStamped, "/localization/acceleration", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Native Loc
        self._imu_pub = self.ros_node.create_publisher(Imu, "/sensing/imu/imu_data", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._ndt_pose_pub = self.ros_node.create_publisher(PoseWithCovarianceStamped, "/localization/pose_estimator/pose_with_covariance", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        #self._vel_pub = self.ros_node.create_publisher(TwistWithCovarianceStamped, "/sensing/vehicle_velocity_converter/twist_with_covariance", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._init_pose_pub = self.ros_node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._gnss_pose_pub = self.ros_node.create_publisher(PoseWithCovarianceStamped, "/sensing/gnss/pose_with_covariance", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Publisher for Vehicle Infos
        self._steering_report_publisher = self.ros_node.create_publisher(SteeringReport, "/vehicle/status/steering_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._velocity_report_publisher = self.ros_node.create_publisher(VelocityReport, "/vehicle/status/velocity_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        #self._actuation_report_publisher = self.ros_node.create_publisher(ActuationStatusStamped, "/vehicle/status/actuation_status", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Publisher & Subscriber for Goal Planning
        self._goal_publisher = self.ros_node.create_publisher(PoseStamped, "/rviz/routing/rough_goal", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self._engage_publisher = self.ros_node.create_publisher(Engage, "/autoware/engage", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self_autoware_mode_subscriber = self.ros_node.create_subscription(OperationModeState, "/system/operation_mode/state", self._operation_mode_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        # Services for Goal Publishing
        self._route_service_client = self.ros_node.create_client(SetRoutePoints, "/api/routing/change_route_points")
        
        # Subscriber for Control
        self._control_subscriber = self.ros_node.create_subscription(Control, "/control/command/control_cmd", self._vehicle_control_cmd_callback, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))

        self._object_publisher = self.ros_node.create_publisher(PredictedObjects, "/perception/object_recognition/objects", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))

        # V2X Traffic Light
        self._lanelet_map_subscriber = self.ros_node.create_subscription(MarkerArray, "/map/vector_map_marker", self._get_traffic_lights_from_lanelet, qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._external_traffic_light_publisher = self.ros_node.create_publisher(TrafficLightGroupArray, "/perception/traffic_light_recognition/traffic_signals", qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))

        self._camera_pub = self.ros_node.create_publisher(Image, "/sensing/camera/traffic_light/image_raw", qos_profile=qos_profile_sensor_data)


        # --End of Autoware Stuff--
        self._client = None
        self._vehicle = None
        self._world = None
        self._map = None
        # TODO(GEMB): Integrate later again, this stops the node until autowre is ready and launched
        # wait_for_message(self.ros_node, "/carla/hero/status", Bool)

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,))
        self.spin_thread.start()

    def setup(self, path_to_conf_file):
        self.track = Track.MAP
        
        # Autoware Priviliged Stuff
        self._client = CarlaDataProvider.get_client()
        self._vehicle = CarlaDataProvider.get_hero_actor()
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()
        self._awp_converter = AutowareConverter(self._vehicle, self._world)
        self._last_control = carla.VehicleControl(throttle=0.0)
        self._route_index = 1 # Start with 1, because 0 is the start point
        self._last_published_route_index = None
        self._published_latest = False
        self._goal_mod_failure_counter = 0
        self._is_autonomous = False

        self._traffic_light_ids = set()  
        self._publish_cam_image = False # For Debugging
    
    def sensors(self) -> list:
        # The privileged expert using Autoware doesn't need any sensors
        sensors = []
        if self._publish_cam_image:
            sensors.append({ "type": "sensor.camera.rgb", "id": "CAMERA_front",
                "x": 0.0, "y": 0.0, "z": 2.0, 
                "roll": 0.0, "pitch": 00.0, "yaw": 0.0, "width": 600 , "height": 300,
                "fov": 90.0})
            self._camera_height = next(s for s in sensors if s["type"] == "sensor.camera.rgb")["height"]
            self._camera_width = next(s for s in sensors if s["type"] == "sensor.camera.rgb")["width"]
            self._camera_fov = next(s for s in sensors if s["type"] == "sensor.camera.rgb")["fov"]

        return sensors

    # After initial initialization the run step method should 
    # wait for the next control signal 

    def run_step(self, input_data, timestamp):
        timestamp = carla_timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
        # Autoware E2E


        # Autoware Priviliged Publisher
        time_sec = int(timestamp)
        time_nsec = int((timestamp - time_sec) * 1e9)
        ros_time = Clock()
        ros_time.clock.sec = time_sec
        ros_time.clock.nanosec = time_nsec
        self._clock_pub.publish(ros_time)
        
        # Get Infos from Vehicle
        position = self._awp_converter._get_localization()
        velocity = self._awp_converter._get_twist()
        loc_acc = self._awp_converter._get_acceleration()

        # TF
        tf_msg = TFMessage()
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp.sec = time_sec
        transform_stamped.header.stamp.nanosec = time_nsec
        transform_stamped.header.frame_id = "map"
        transform_stamped.child_frame_id = "base_link"
        transform_stamped.transform.translation = Vector3(**position["position"])
        transform_stamped.transform.rotation = Quaternion(**position["orientation"])
        tf_msg.transforms.append(transform_stamped)
        self._tf_publisher.publish(tf_msg)

        # GT pose
        localization_msg = Odometry()
        localization_msg.header.stamp.sec = time_sec
        localization_msg.header.stamp.nanosec = time_nsec
        localization_msg.header.frame_id = "map"
        localization_msg.child_frame_id = "base_link"
        localization_msg.pose.pose.position = Point(**position["position"])
        localization_msg.pose.pose.orientation = Quaternion(**position["orientation"])
        localization_msg.twist.twist.linear = Vector3(**velocity["position"])
        localization_msg.twist.twist.angular = Vector3(
            x=velocity["orientation"]["roll"],
            y=velocity["orientation"]["pitch"],
            z=velocity["orientation"]["yaw"],
        )
        self._localization_publisher.publish(localization_msg)

        loc_state_msg = LocalizationInitializationState()
        loc_state_msg.stamp.sec = time_sec
        loc_state_msg.stamp.nanosec = time_nsec
        loc_state_msg.state = LocalizationInitializationState.INITIALIZED
        self._localization_init_state_publisher.publish(loc_state_msg)

        # Acceleration
        loc_acc_msg = AccelWithCovarianceStamped()
        loc_acc_msg.header.stamp.sec = time_sec
        loc_acc_msg.header.stamp.nanosec = time_nsec
        loc_acc_msg.header.frame_id = "base_link"
        loc_acc_msg.accel.accel.linear = Vector3(**loc_acc["position"])
        self._localization_acceleration_publisher.publish(loc_acc_msg)

        # Steering Angle
        steering_angle = self._awp_converter._get_steering()
        steering_angle_msg = SteeringReport()
        steering_angle_msg.stamp.sec = time_sec
        steering_angle_msg.stamp.nanosec = time_nsec
        steering_angle_msg.steering_tire_angle = steering_angle
        self._steering_report_publisher.publish(steering_angle_msg)

        # Velocity Report
        velocity_report_msg = VelocityReport()
        velocity_report_msg.header.stamp.sec = time_sec
        velocity_report_msg.header.stamp.nanosec = time_nsec
        velocity_report_msg.header.frame_id = 'base_link'
        velocity_report_msg.longitudinal_velocity = velocity["position"]["x"]
        self._velocity_report_publisher.publish(velocity_report_msg)

        # Goal
        self.publish_global_plan(position["position"])
        # Engage if goal is published and not autonomous
        if self._published_latest and not self._is_autonomous:
            #self._engage_publisher.publish(Engage(engage=True))
            print('enage')

        # IMU
        imu_msg = Imu()
        imu_msg.header.stamp.sec = time_sec
        imu_msg.header.stamp.nanosec = time_nsec
        imu_msg.header.frame_id = "imu_link"
        imu_msg.angular_velocity = Vector3(
            x=velocity["orientation"]["roll"],
            y=velocity["orientation"]["pitch"],
            z=velocity["orientation"]["yaw"]
        )
        imu_msg.linear_acceleration = Vector3(**loc_acc["position"])
        self._imu_pub.publish(imu_msg)
        
        # -- Sensors --

        if self._publish_cam_image:
            cam_msg = Image()
            cam_msg.header.stamp.sec = time_sec
            cam_msg.header.stamp.nanosec = time_nsec
            cam_msg.header.frame_id = "camera_front_optical_link"
            cam_msg.height = self._camera_height  # Must be set, e.g., 600
            cam_msg.width = self._camera_width 
            cam_msg.encoding = "bgra8"
            cam_msg.is_bigendian = 0
            cam_msg.step = self._camera_width * 4 
            cam_msg.data = memoryview(input_data['CAMERA_front'][1]).tobytes()
            self._camera_pub.publish(cam_msg)

        # GT Objects
        objects_msg = self._awp_converter.create_predicted_object_message()
        objects_msg.header.stamp.sec = time_sec
        objects_msg.header.stamp.nanosec = time_nsec
        objects_msg.header.frame_id = "map"

        self._object_publisher.publish(objects_msg)

        # We set all autoware traffic lights to the state of the currently affecting traffic light
        traffict_light_state = self._awp_converter.get_current_traffic_light_state()
        traffic_light_msg = TrafficLightGroupArray()
        traffic_light_msg.stamp.sec = time_sec
        traffic_light_msg.stamp.nanosec = time_nsec
        for traffic_light_id in self._traffic_light_ids:
            traffic_light_group = TrafficLightGroup()
            traffic_light_group.traffic_light_group_id = traffic_light_id
            traffic_light_group.elements.append(TrafficLightElement())
            traffic_light_group.elements[0].color = traffict_light_state
            traffic_light_group.elements[0].shape = 1
            traffic_light_group.elements[0].status = 2
            traffic_light_group.elements[0].confidence = 1.0 
            traffic_light_msg.traffic_light_groups.append(traffic_light_group)           
        self._external_traffic_light_publisher.publish(traffic_light_msg) 
        
        

        # wait 100ms
        #if time_sec > 5: time.sleep(0.02)

        try:
            control_timestamp, control = self._control_queue.get(True, 0.05)
            self._last_control = control
        except queue.Empty:
            control_timestamp, control = timestamp, self._last_control

        carla_timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
        if abs(control_timestamp - carla_timestamp) > EPSILON:
            print(
                "\033[93mWARNING: Expecting a vehicle command with timestamp {} but the timestamp received was {} .\033[0m".format(carla_timestamp, control_timestamp),
                 sep=" ")
        #print('FINAL!!!', control)
        return control

    @staticmethod
    def get_ros_version():
        return AutowarePriviligedAgent.ROS_VERSION

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        ds_ids = downsample_route(global_plan_world_coord, 120)
        self._global_plan_world_coord = [(global_plan_world_coord[x][0], global_plan_world_coord[x][1]) for x in ds_ids]
        self._global_plan = [global_plan_gps[x] for x in ds_ids]

    def publish_global_plan(self, current_position):
        if self._global_plan_world_coord and self._route_index < len(self._global_plan_world_coord) -1:
            goal_wp = self._global_plan_world_coord[self._route_index]
            goal_pose_dict = BridgeHelper.carla2ros_pose(
                    goal_wp[0].location.x, goal_wp[0].location.y, goal_wp[0].location.z,
                    np.deg2rad(goal_wp[0].rotation.roll), np.deg2rad(goal_wp[0].rotation.pitch), np.deg2rad(goal_wp[0].rotation.yaw),
                    to_quat=True
                )
            # Only Publish a new goal when distance to last goal smaller 50 meter
            dist = math.sqrt(
                (goal_pose_dict["position"]["x"] - current_position["x"])**2 +
                (goal_pose_dict["position"]["y"] - current_position["y"])**2 +
                (goal_pose_dict["position"]["z"] - current_position["z"])**2
            )
            print("Distance to goal: ", dist)
            if dist < 50.0 or self._goal_mod_failure_counter > 5: 
                self._route_index += 1
                self._published_latest = False
                self._goal_mod_failure_counter = 0

            # Publish only the first Checkpoint as a goal 
            # Use reruting service to set the next goals
                
            if not self._published_latest:
                next_wp = self._global_plan_world_coord[self._route_index]
                next_pose_dict = BridgeHelper.carla2ros_pose(
                    next_wp[0].location.x, next_wp[0].location.y, next_wp[0].location.z,
                    np.deg2rad(next_wp[0].rotation.roll),
                    np.deg2rad(next_wp[0].rotation.pitch),
                    np.deg2rad(next_wp[0].rotation.yaw),
                    to_quat=True
                )

                service_request = SetRoutePoints.Request()
                service_request.header.frame_id = "map"
                service_request.option.allow_goal_modification = True
                service_request.option.allow_while_using_route = False
                service_request.goal = Pose(
                    position=Point(**next_pose_dict["position"]),
                    orientation=Quaternion(**next_pose_dict["orientation"])
                )
                future = self._route_service_client.call_async(service_request)
                future.add_done_callback(self._handle_service_response)
        else:
            if not self._published_latest:
                next_wp = self._global_plan_world_coord[-1]
                next_pose_dict = BridgeHelper.carla2ros_pose(
                    next_wp[0].location.x, next_wp[0].location.y, next_wp[0].location.z,
                    np.deg2rad(next_wp[0].rotation.roll),
                    np.deg2rad(next_wp[0].rotation.pitch),
                    np.deg2rad(next_wp[0].rotation.yaw),
                    to_quat=True
                )

                service_request = SetRoutePoints.Request()
                service_request.header.frame_id = "map"
                service_request.option.allow_goal_modification = True
                service_request.option.allow_while_using_route = False
                service_request.goal = Pose(
                    position=Point(**next_pose_dict["position"]),
                    orientation=Quaternion(**next_pose_dict["orientation"])
                )
                future = self._route_service_client.call_async(service_request)
                future.add_done_callback(self._handle_service_response)


            
    def _handle_service_response(self, future):
        try:
            self._published_latest = future.result().status.success
            if not self._published_latest:
                self._goal_mod_failure_counter += 1
            
            # If Error is route not set yet publish a new goal
            if future.result().status.message == 'The route is not set yet.':
                self._published_latest = True
                next_wp = self._global_plan_world_coord[self._route_index]
                next_pose_dict = BridgeHelper.carla2ros_pose(
                    next_wp[0].location.x, next_wp[0].location.y, next_wp[0].location.z,
                    np.deg2rad(next_wp[0].rotation.roll),
                    np.deg2rad(next_wp[0].rotation.pitch),
                    np.deg2rad(next_wp[0].rotation.yaw),
                    to_quat=True
                )
                
                goal_pose_stamped = PoseStamped()
                goal_pose_stamped.header.frame_id = "map"  # or whatever your frame is
                goal_pose_stamped.pose = Pose(
                    position=Point(**next_pose_dict["position"]),
                    orientation=Quaternion(**next_pose_dict["orientation"])
                )
                print("Publishing goal")
                self._goal_publisher.publish(goal_pose_stamped)
            
            print('Response:', future.result().status )
        except Exception as e:
            print('Service call failed:', e)
    
    def _vehicle_control_cmd_callback(self, control_msg):
        control_timestamp, control = self._awp_converter.convert_control(control_msg)


        # Checks that the received control timestamp is not repeated.
        if self._last_control_timestamp is not None and abs(self._last_control_timestamp - control_timestamp) < EPSILON:
            print(
                "\033[93mWARNING 11111: A new vehicle command with a repeated timestamp has been received {} .\033[0m".format(control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        # Checks that the received control timestamp is the expected one.
        # We need to retrieve the simulation time directly from the CARLA snapshot instead of using the GameTime object to avoid
        # a race condition between the execution of this callback and the update of the GameTime internal variables.
        carla_timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
        if abs(control_timestamp - carla_timestamp) > EPSILON:
            print(
                "\033[93mWARNING2222: Expecting a vehicle command with timestamp {} but the timestamp received was {} .\033[0m".format(carla_timestamp, control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        self._last_control_timestamp = control_timestamp
        try:
            self._control_queue.put_nowait((control_timestamp, control))
        except queue.Full:
            print(
                "\033[93mWARNING3333: A new vehicle command has been received while the previous one has not been yet processed.\033[0m",
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")

    def _vehicle_control_cmd_callback2(self, aw_ackermann_control_command_msg):
        carla_ackermann_control = AckermannDrive()
        carla_ackermann_control.steering_angle = aw_ackermann_control_command_msg.lateral.steering_tire_angle * 1.2
        carla_ackermann_control.steering_angle_velocity = aw_ackermann_control_command_msg.lateral.steering_tire_rotation_rate * 1.2
        carla_ackermann_control.speed = aw_ackermann_control_command_msg.longitudinal.speed
        carla_ackermann_control.acceleration = aw_ackermann_control_command_msg.longitudinal.acceleration
        carla_ackermann_control.jerk = aw_ackermann_control_command_msg.longitudinal.jerk
        self._control_publiher.publish(carla_ackermann_control)

    def _vehicle_control_cmd_callback3(self, control_msg):
        
        control_timestamp, control = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds, carla.VehicleControl(
            steer = control_msg.steer,
            throttle = control_msg.throttle,
            brake = control_msg.brake,
        )


        # Checks that the received control timestamp is not repeated.
        if self._last_control_timestamp is not None and abs(self._last_control_timestamp - control_timestamp) < EPSILON:
            print(
                "\033[93mWARNING 11111: A new vehicle command with a repeated timestamp has been received {} .\033[0m".format(control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        # Checks that the received control timestamp is the expected one.
        # We need to retrieve the simulation time directly from the CARLA snapshot instead of using the GameTime object to avoid
        # a race condition between the execution of this callback and the update of the GameTime internal variables.
        carla_timestamp = CarlaDataProvider.get_world().get_snapshot().timestamp.elapsed_seconds
        if abs(control_timestamp - carla_timestamp) > EPSILON:
            print(
                "\033[93mWARNING2222: Expecting a vehicle command with timestamp {} but the timestamp received was {} .\033[0m".format(carla_timestamp, control_timestamp),
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
            return

        self._last_control_timestamp = control_timestamp
        try:
            self._control_queue.put_nowait((control_timestamp, control))
        except queue.Full:
            print(
                "\033[93mWARNING3333: A new vehicle command has been received while the previous one has not been yet processed.\033[0m",
                "\033[93mThis vehicle command will be ignored.\033[0m",
                sep=" ")
        
    def _acc_callback(self, acc_msg):
        self._awp_converter.getAcc(acc_msg)
    
    def _operation_mode_callback(self, operation_mode_msg):
        if operation_mode_msg.mode == OperationModeState.AUTONOMOUS:
            self._is_autonomous = True
        else:
            self._is_autonomous = False

    def _get_traffic_lights_from_lanelet(self, lanelet_marker_msg):
        print("Fetch Traffic Lights from Lanelet")
        pattern = re.compile(r"TLRegElemId:(\d+)")
        for marker in lanelet_marker_msg.markers:
            if marker.text:
                match = pattern.search(marker.text)
                if match:
                    self._traffic_light_ids.add(int(match.group(1)))

    def destroy(self):
        """
        Destroy (clean-up) the agent
        :return:
        """
        self.ros_node.destroy_node()
        rclpy.shutdown()
        self.spin_thread.join()

        super(AutowarePriviligedAgent, self).destroy()