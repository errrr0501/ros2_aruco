"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:1000 Mb/s
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMa1000 Mb/srkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
from ros2_aruco import transformations

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from ros2_aruco_interfaces.srv import GetMaskImage
from math import *

from tf2_ros import TransformBroadcaster

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

class SingleArucoSetTf(rclpy.node.Node):

    def __init__(self):
        super().__init__('set_aruco_tf')

        # Declare and read parameters
        self.declare_parameter("marker_size", .05)
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")
        self.declare_parameter("image_topic", "techman_image")#???
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("camera_frame", None)

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        # self.armarker_info_topic = self.get_parameter("marker_info_topic").get_parameter_value().string_value

        self.corners = []
        self.marker_ids = []
        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid optarmarker_info_topicions: {}".format(options))

        # Set up subscriptions
        # self.info_sub = self.create_subscription(CameraInfo,
        #                                          info_topic,
        #                                          self.info_callback,
        #                                          qos_profile_sensor_data)

        self.create_subscription(Image, image_topic,
                                 self.image_callback, qos_profile_sensor_data)

        # Set up publishers
        # self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        # self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        # Initialize the transform broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)


        # Set up fields for camera parameters
        # self.info_msg = None
        # self.intrinsic_mat = None
        # self.distortion = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

        self.intrinsic_mat = np.reshape(np.array([2796.9298, 0.0000, 1299.0794, 
                                                    0.0000, 2797.2478, 994.3013, 
                                                    0.0000, 0.0000, 1.0000]), (3, 3))
        self.distortion = np.array([0.1039, -0.6948, 0.0027, -0.0009, 0.8045])
        self.cv_image = None

    # def info_callback(self, info_msg):
    #     self.info_msg = info_msg
    #     self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
    #     self.distortion = np.array(self.info_msg.d)
    #     # Assume that camera parameters will remain the same...
    #     self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        self.get_logger().warn("------------------------------ received!")
        # if self.info_msg is None:
        #     self.get_logger().warn("No camera info has been received!")
        #     return
        # cv_image = self.bridge.imgmsg_to_cv2(img_msg,
        #                                      desired_encoding='mono8')
        cv_image = self.bridge.imgmsg_to_cv2(img_msg,
                                             desired_encoding='passthrough')
        # self.cv_image = self.bridge.imgmsg_to_cv2(img_msg,
        #                                      desired_encoding='bgr8')


        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame is None:
            # markers.header.frame_id = self.info_msg.header.frame_id
            # pose_array.header.frame_id = self.info_msg.header.frame_id
            markers.header.frame_id = 'tm_vision'
            pose_array.header.frame_id = 'tm_vision'
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame
            
            
        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        self.corners, self.marker_ids, rejected = cv2.aruco.detectMarkers(cv_image,
                                                                self.aruco_dictionary,
                                                                parameters=self.aruco_parameters)

        if self.marker_ids is not None:
            self.get_logger().warn("------------------------------ ar_marker!")
            self.get_marker = True

            if cv2.__version__ > '4.0.0':
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.corners,
                                                                      self.marker_size, self.intrinsic_mat,
                                                                      self.distortion)
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(self.corners,
                                                                   self.marker_size, self.intrinsic_mat,
                                                                   self.distortion)
            # self.cv_image = cv2.aruco.drawAxis(self.cv_image, self.intrinsic_mat, self.distortion, rvecs[0], tvecs[0], 0.02)
            # cv2.imshow('QueryImage', self.cv_image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            for i, marker_id in enumerate(self.marker_ids):
                pose = TransformStamped()

                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'tm_vision'
                pose.child_frame_id = 'ar_marker'
                pose.transform.translation.x = tvecs[i][0][0]
                pose.transform.translation.y = tvecs[i][0][1]
                pose.transform.translation.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = transformations.quaternion_from_matrix(rot_matrix)

                pose.transform.rotation.x = quat[0]
                pose.transform.rotation.y = quat[1]
                pose.transform.rotation.z = quat[2]
                pose.transform.rotation.w = quat[3]

                angle = transformations.euler_from_quaternion(quat)
                print(angle[0]*180/pi)
                print(angle[1]*180/pi)
                print(angle[2]*180/pi)
                # pose_array.poses.append(pose)
                # markers.poses.append(pose)
                # markers.marker_ids.append(marker_id[0])

            # self.poses_pub.publish(pose_array)
            self.tf_broadcaster.sendTransform(pose)
            # self.markers_pub.publish(markers)
        else:
            self.get_marker = False

        try:
            t = self.tf_buffer.lookup_transform(
                'base',
                'ar_marker',
                rclpy.time.Time())
            print(t)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base to ar_marker: {ex}')
            return


        #flange
        # 0.12167
        # 0.52174
        # 0.30864
        # -179.07
        # -0.61
        # 179.97
        
        #camera 
        # 0.12179
        # 0.5963
        # 0.26476
        # 177.74
        # 1.5
        # -0.19

def main():
    rclpy.init()
    node = SingleArucoSetTf()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
