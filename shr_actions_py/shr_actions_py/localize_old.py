﻿import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node

from shr_msgs.action import LocalizeRequest
from nav2_msgs.msg import ParticleCloud

import numpy as np
import math
import time
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import tf2_ros
import tf_transformations as tr
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.callback_groups import ReentrantCallbackGroup


class LocalizationActionServer(Node):

    def __init__(self):
        super().__init__('Localization_action_server')
        self.nav2_to_goal_client = None
        self._action_server = ActionServer(
            self,
            LocalizeRequest,
            'localize',
            self.execute_callback,
            goal_callback=self.goal_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback)  # Add goal_callback

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
            callback_group=ReentrantCallbackGroup())

        ## get the pose
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # For localization

        self.vel_pub = self.create_publisher(Twist, os.getenv("cmd_vel"), 10)
        self.subscriber = self.create_subscription(ParticleCloud, 'particle_cloud', self.particles_callback,
                                                   qos_profile, callback_group=ReentrantCallbackGroup())

        self.publisher_initial_pose = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

# For April tags
        self.subscription = self.create_subscription(AprilTagDetectionArray, '/apriltag_detections',
                                                     self.apriltag_callback, 10, callback_group=ReentrantCallbackGroup())
        self.aptags_detected = False
        self.buffer = tf2_ros.Buffer()
        self.tf_buffer = self.buffer
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self, spin_thread=True)
        # self.used_apriltags = [0, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 29]  # add the apriltag ids that you used
        self.used_apriltags = [0, 2, 3, 4, 6, 7, 8, 25, 15, 29, 28, 203]
        # add the apriltag ids that you used
        self.transform_aptag_in_cam_dict = {}  # location of apriltags in camera frame
        self.transform_aptag_in_world_dict = {}  # global location of apriltags
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.closest_aptag = None

        self.result_future = None
        self.max_weight = 0
        self.time_out = 200
        self.get_tf_info = True
        self.successfully_localized = False
        self.aptags_detected_inside_callback = False

        self.get_transform_matrix_aptags_in_world_from_tf()
        # self.min_range = None
        self.scan = False
        self.threshold = -0.5 # set based on worst case in robot

    def scan_callback(self, msg):
        if msg:
            self.scan = True
            back_start = int(3.5*(len(msg.ranges)/8)) #0
            back_end = int(4.5*(len(msg.ranges)/8)-1)
            self.min_range = min(msg.ranges)
            ## diff drive so can only go back and front, rotation is a bit of axis
            self.min_range_back = min(msg.ranges[back_start:back_end])
            self.min_range_left_quad = min(msg.ranges[int(3*(len(msg.ranges)/4)) : int(4*(len(msg.ranges)/4)) - 1])
            self.min_range_right_quad = min(msg.ranges[0: int((len(msg.ranges)/4)) -1])

            indices = list(range(int(7*(len(msg.ranges)/8)), len(msg.ranges)-1)) + list(range(0, int((len(msg.ranges)/8)-1)))
            self.min_range_front = min([msg.ranges[i] for i in indices])
            # self.min_range_front = min(msg.ranges[back_start:back_start])

    def cancel_callback(self, goal_handle):
        # Your cancellation logic here
        self.get_logger().info('Goal cancelled')
        self.goal_cancel = True
        return CancelResponse.ACCEPT

    ##### Localization Part #####
    def publish_tf(self, x, y, z, rot_mat, child_frame_id, frame_id):
        quat = Quaternion()
        quat_ = self.rotation_matrix_to_quaternion(np.array(rot_mat))
        quat.x = quat_[0]
        quat.y = quat_[1]
        quat.z = quat_[2]
        quat.w = quat_[3]

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = frame_id
        static_transform.child_frame_id = child_frame_id

        static_transform.transform.translation.x = x  # Set translation values
        static_transform.transform.translation.y = y
        static_transform.transform.translation.z = z  # meters

        static_transform.transform.rotation = quat
        self.tf_broadcaster.sendTransform(static_transform)
        # print(f'publishing child {child_frame_id} from {frame_id}')

    def get_dist(self, x, y, z):
        return np.linalg.norm([x, y, z])

    def apriltag_callback(self, msg):
        ### THIS SHOULD HAVE A FLAG IF APRILTAG CALLBACK CALCULATE TF IS TRUE THEN DO THE CLACULATION BUT FIRST IJUST WANT TO CHECK IF THERE ARE TAGS DETECTED

        if msg.detections:
            min_distance = np.inf
            if self.get_tf_info:
                self.transform_aptag_in_cam_dict = {}
                source_frame = msg.header.frame_id  # to
                # source_frame = "base_link"  # to

                try:
                    for at in msg.detections:
                        if at.id == 203:
                            continue
                        self.aptags_detected = True
                        frame = "tag_" + str(at.id)  # from
                        transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                                         timeout=rclpy.duration.Duration(
                                                                             seconds=1000.0))

                        dist = self.get_dist(transformation.transform.translation.x,
                                             transformation.transform.translation.y,
                                             transformation.transform.translation.z)
                        if dist < min_distance:
                            min_distance = dist
                            self.closest_aptag = at.id

                        translation = tr.translation_matrix(
                            [transformation.transform.translation.x, transformation.transform.translation.y,
                             transformation.transform.translation.z])
                        rotation = tr.quaternion_matrix(
                            [transformation.transform.rotation.x, transformation.transform.rotation.y,
                             transformation.transform.rotation.z, transformation.transform.rotation.w])
                        # print('source', source_frame, 'frame', frame, 'rotation', rotation)
                        # Get the homogeneous transformation matrix
                        transform_aptag_in_cam = np.dot(translation, rotation)

                        self.transform_aptag_in_cam_dict[at.id] = transform_aptag_in_cam
                        # print('self.transform_aptag_in_cam_dict[at.id]', self.transform_aptag_in_cam_dict[at.id])
                        # self.get_logger().info(f'transform ready from {frame} to {source_frame}')
                        # print("aptag detected!!!!!!")
                        self.aptags_detected_inside_callback = True
                        # print('********** apriltag detectedd: ' + str(at.id))

                except Exception as ex:
                    # self.aptags_detected_inside_callback = False
                    self.get_logger().info(f'Error ***************: {ex}')

        else:
            # pass
            print('No aptags from callback')
            self.aptags_detected = False
            self.aptags_detected_inside_callback = False

    
    def get_transform_matrix_aptags_in_world_from_tf(self):
        self.transform_aptag_in_cam_dict = {}
        for aptag in self.used_apriltags:

            str_aptag = str(aptag)
            source_frame = "map"  # to
            frame = "aptag_" + str_aptag  # from

            try:
                transformation = self.tf_buffer.lookup_transform(source_frame, frame, rclpy.time.Time(),
                                                                 timeout=rclpy.duration.Duration(seconds=5.0))

                translation = tr.translation_matrix(
                    [transformation.transform.translation.x, transformation.transform.translation.y,
                     transformation.transform.translation.z])
                rotation = tr.quaternion_matrix(
                    [transformation.transform.rotation.x, transformation.transform.rotation.y,
                     transformation.transform.rotation.z, transformation.transform.rotation.w])
                
                
                ### debugging
                # SAME THING
                # rotation__ = self.quaternion_rotation_matrix(transformation.transform.rotation)
                

                # print("tr quat rot", rotation)
                # print("quat rot",rotation__ )
                
                
                # Get the homogeneous transformation matrix
                transform_aptag_in_world = np.dot(translation, rotation)

                self.transform_aptag_in_world_dict[aptag] = transform_aptag_in_world

                self.get_logger().info(f'transform ready from {frame} to {source_frame}')

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

    # def rotation_matrix_to_quaternion(self, R):
    #     # Convert a 3x3 rotation matrix to a Quaternion
    #     trace = np.trace(R)
    #     if trace > 0:
    #         S = 2.0 * math.sqrt(trace + 1.0)
    #         qw = 0.25 * S
    #         qx = (R[2, 1] - R[1, 2]) / S
    #         qy = (R[0, 2] - R[2, 0]) / S
    #         qz = (R[1, 0] - R[0, 1]) / S
    #     elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
    #         S = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
    #         qw = (R[2, 1] - R[1, 2]) / S
    #         qx = 0.25 * S
    #         qy = (R[0, 1] + R[1, 0]) / S
    #         qz = (R[0, 2] + R[2, 0]) / S
    #     elif R[1, 1] > R[2, 2]:
    #         S = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
    #         qw = (R[0, 2] - R[2, 0]) / S
    #         qx = (R[0, 1] + R[1, 0]) / S
    #         qy = 0.25 * S
    #         qz = (R[1, 2] + R[2, 1]) / S
    #     else:
    #         S = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    #         qw = (R[1, 0] - R[0, 1]) / S
    #         qx = (R[0, 2] + R[2, 0]) / S
    #         qy = (R[1, 2] + R[2, 1]) / S
    #         qz = 0.25 * S

    #     return [qx, qy, qz, qw]
            
    def rotation_matrix_to_quaternion(self, R):
        t = np.matrix.trace(R)
        q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        if (t > 0):
            t = np.sqrt(t + 1)
            q[3] = 0.5 * t
            t = 0.5 / t
            q[0] = (R[2, 1] - R[1, 2]) * t
            q[1] = (R[0, 2] - R[2, 0]) * t
            q[2] = (R[1, 0] - R[0, 1]) * t

        else:
            i = 0
            if (R[1, 1] > R[0, 0]):
                i = 1
            if (R[2, 2] > R[i, i]):
                i = 2
            j = (i + 1) % 3
            k = (j + 1) % 3

            t = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[3] = (R[k, j] - R[j, k]) * t
            q[j] = (R[j, i] + R[i, j]) * t
            q[k] = (R[k, i] + R[i, k]) * t

        return q

    def particles_callback(self, msg):
        max_weight = 0.0  # Initialize with a double value
        particles_count = 0.0

        for particle in msg.particles:
            weight = particle.weight

            if weight > max_weight:
                max_weight = weight
                particles_count += particles_count

        self.max_weight = max_weight

        if particles_count > 0:
            self.get_logger().info('no particles')

        # print(self.max_weight, '**************************')

    def publish_pose(self, robot_pose_aptags, rotation_matrix):
        robot_pose = PoseWithCovarianceStamped()

        # self.get_logger().info(f'Publishiungg g POSEEEEE')

        quat = Quaternion()
        quat_ = self.rotation_matrix_to_quaternion(np.array(rotation_matrix))
        quat.x = quat_[0]
        quat.y = quat_[1]
        quat.z = quat_[2]
        quat.w = quat_[3]

        robot_pose.header.stamp = self.get_clock().now().to_msg()

        robot_pose.header.frame_id = "map"  # or frame_link

        robot_pose.pose.pose.position.x = robot_pose_aptags[0]
        robot_pose.pose.pose.position.y = robot_pose_aptags[1]
        robot_pose.pose.pose.orientation = quat
        robot_pose.pose.covariance = self.covariance

        self.publisher_initial_pose.publish(robot_pose)
        # self.get_logger().info(f'FINISH   Publishingg POSEEEEE')

    def transform_cam_world_frame(self):
        robot_position = []
        transform_aptag_in_cam_dict = self.transform_aptag_in_cam_dict
        transform_aptag_in_world_dict = self.transform_aptag_in_world_dict

        for aptag in transform_aptag_in_cam_dict.keys():
            # print(aptag)te
            t_apriltag_to_world = transform_aptag_in_world_dict[aptag]
            t_apriltag_in_camera = transform_aptag_in_cam_dict[aptag]

            t_cam_in_world = np.dot(t_apriltag_to_world, np.linalg.inv(t_apriltag_in_camera))

            ## get it from  ros2 run tf2_ros tf2_echo camera_color_optical_frame base_link
            transform_cam_to_base_link = np.array([[-0.397, -0.062, -0.916,  1.212],
                                                   [-0.225,  0.974,  0.031, -0.033],
                                                   [0.890,  0.218, -0.401,  0.492],
                                                   [0.0, 0.0, 0.0, 1.0]])

            t_robot_in_world = np.dot(t_cam_in_world, np.linalg.inv(transform_cam_to_base_link))


            robot_x = t_robot_in_world[0, 3]
            robot_y = t_robot_in_world[1, 3]
            robot_z = t_robot_in_world[2, 3]

            # Append the column vector to the robot_position array
            robot_position.append([robot_x, robot_y, robot_z])

            if aptag == self.closest_aptag:
                # no average for theta jst take the one of the closest aptag
                rotation_matrix = t_robot_in_world[:3, :3]


        # Convert the robot_position list to a NumPy array
        robot_position_array = np.array(robot_position)

        # Compute the mean for each row to get average of position computed from different aptags
        mean_values = np.mean(robot_position_array, axis=0)

        print("rotation matrix ******", rotation_matrix)
        print("rotation matrix ******", mean_values)
        return mean_values, rotation_matrix

    def localize(self):
        # LOCALIZE
        start_time = time.time()
        speed = 3.14 / 8.0
        msg = Twist()

        # CHECK IF THERE ARE ANY APRILTAGS
        if not self.aptags_detected:
            # rotate until you find one
            while time.time() - start_time < self.time_out:
                self.get_logger().info('no aptags detected will start looking for one')
                # change this to VECTOR FIELD HISTOGRAM exploration
                if self.scan and  self.min_range > self.threshold:
                    msg.angular.z = speed
                    self.vel_pub.publish(msg)
                else:
                    # stop the robot casue it's not safe
                    msg.angular.z = 0
                    # move in the direction with no obstacle
                    if self.min_range_back > self.threshold: ## clear in the back, move backwards
                        msg.linear.x = -0.1
                        self.vel_pub.publish(msg)
                    elif self.min_range_front > self.threshold:
                        msg.linear.x = -0.1
                    # elif   check what's needed here from running at sajay
                    self.vel_pub.publish(msg)
                if self.aptags_detected_inside_callback:
                    # STOP
                    msg.angular.z = 0.0
                    self.vel_pub.publish(msg)
                    # localize
                    self.get_tf_info = True
                    # print("published_pose)))))))))))))))))))" )
                    if self.transform_aptag_in_cam_dict and self.transform_aptag_in_world_dict:
                        # publish the pose that can be subscribed to by nav2 for initial position or we can change setup to service
                        robot_pose_aptags, rotation_matrix = self.transform_cam_world_frame()

                        self.publish_pose(robot_pose_aptags, rotation_matrix)
                        # print("published_pose")
                        self.publish_tf(robot_pose_aptags[0], robot_pose_aptags[1], robot_pose_aptags[2],
                                        rotation_matrix,
                                        'base_link', 'map')
                        self.successfully_localized = True

                        return

                    else:
                        # self.successfully_localized = False
                        if not self.transform_aptag_in_cam_dict:
                            self.get_logger().info('NO apriltags detected')
                        if not self.transform_aptag_in_world_dict:
                            self.get_transform_matrix_aptags_in_world_from_tf()
                            self.get_logger().info('NO transform_aptag_in_world_dict')
                        # if self.cam_to_base_link is None:
                        #     self.get_logger().info('NO transformation cam_to_base_link')
                        # return
        else:
            # localize
            self.get_tf_info = True
            # print("published_pose)))))))))))))))))))" )
            if self.transform_aptag_in_cam_dict and self.transform_aptag_in_world_dict:
                # publish the pose that can be subscribed to by nav2 for initial position or we can change setup to service
                robot_pose_aptags, rotation_matrix = self.transform_cam_world_frame()

                self.publish_pose(robot_pose_aptags, rotation_matrix)
                # print("published_pose")
                self.publish_tf(robot_pose_aptags[0], robot_pose_aptags[1], robot_pose_aptags[2], rotation_matrix,
                                'base_link', 'map')
                self.successfully_localized = True

                return

            else:
                # self.successfully_localized = False

                if not self.transform_aptag_in_cam_dict:
                    self.get_logger().info('NO apriltags detected')
                if not self.transform_aptag_in_world_dict:
                    self.get_transform_matrix_aptags_in_world_from_tf()
                    self.get_logger().info('NO transform_aptag_in_world_dict')
                # return

                # if self.cam_to_base_link is None:
                #     self.get_logger().info('NO transformation cam_to_base_link')

    ##### Action Server #####

    def particles_callback(self, msg):
        max_weight = 0.0  # Initialize with a double value
        particles_count = 0.0
        for particle in msg.particles:
            weight = particle.weight
            if weight > max_weight:
                max_weight = weight
            particles_count = particles_count + 1
        self.max_weight = max_weight

        # print("1/particles_count" , 1/particles_count)
        if particles_count == 0:
            self.get_logger().info('no particles')

    def goal_callback(self, goal_request):
        # You can add logic here to decide whether to accept or reject the goal.
        # For example, you might check if the goal is valid or if the robot is ready.

        # If you want to accept the goal, return GoalResponse.ACCEPT
        # If you want to reject the goal, return GoalResponse.REJECT

        self.get_logger().info("weblog=" + 'ACCEPTED navigation goal')

        return GoalResponse.ACCEPT

    def feedback_callback(self, msg):
        # self.get_logger().info('Received action feedback message')
        self.feedback = msg.feedback
        return

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        result = LocalizeRequest.Result()

        self.get_logger().info("weblog=" + 'Executing goal...')

        # Perform the navigation and localization logic here.
        # Access the goal from the goal handle

        if goal_handle.request.force_localize:
            self.max_weight = 0.0

        if (self.max_weight >= 0.0015):  # 0.0015):
            self.get_logger().info('Robot is not lost; continuing without localizing')
            goal_handle.succeed()
            result.result = True
            return result

        else:
            self.get_logger().info("weblog=" + 'Robot is lost; Localizing')
            self.localize()
            self.get_logger().info("weblog=" + 'Robot Localized')

        self.get_logger().info('Sending goal')

        # Assuming you successfully navigated and localized, set the goal state to succeeded or abort since the code is already in executing state
        if self.successfully_localized:
            goal_handle.succeed()
            result.result = True
        else:
            goal_handle.abort()
            result.result = False

        # If you want to set the goal state to aborted in case of an error, use:
        # goal_handle.abort(result)
        self.get_logger().info("weblog=" + 'Goal Executed...')

        return result


def main(args=None):
    rclpy.init(args=args)

    loc_action_server = LocalizationActionServer()

    exe = rclpy.executors.MultiThreadedExecutor()
    exe.add_node(loc_action_server)
    while True:
        exe.spin_once(timeout_sec=5.0)

    loc_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
