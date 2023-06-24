"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
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

class MaskColorAndDepthImage(rclpy.node.Node):

    def __init__(self):
        super().__init__('aruco_node')

        # Declare and read parameters
        # self.declare_parameter("marker_size", .053)
        # self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")
        # self.declare_parameter("image_topic", "/camera/color/image_raw")#???
        # self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        # self.declare_parameter("camera_frame", None)
        # self.declare_parameter("depth_image_topic", "/camera/aligned_depth_to_color/image_raw")
        # self.declare_parameter("marker_info_topic", "/aruco_markers")

        self.declare_parameter("marker_size", .053)
        self.declare_parameter("aruco_dictionary_id", "DICT_5X5_250")
        self.declare_parameter("image_topic", "/rgb/image_raw")
        self.declare_parameter("camera_info_topic", "/rgb/camera_info")
        self.declare_parameter("camera_frame", None)
        self.declare_parameter("depth_image_topic", "/depth_to_rgb/image_raw")
        self.declare_parameter("marker_info_topic", "/aruco_markers")

        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value
        dictionary_id_name = self.get_parameter(
            "aruco_dictionary_id").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        depth_image_topic = self.get_parameter("depth_image_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.armarker_info_topic = self.get_parameter("marker_info_topic").get_parameter_value().string_value

        self.corners = []
        self.marker_ids = []
        self.cv_image = None
        self.depth_image = None
        self.masked_image = ['initial']
        self.get_marker = False
        self.get_mask_image = False

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {}".format(dictionary_id_name))
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo,
                                                 info_topic,
                                                 self.info_callback,
                                                 qos_profile_sensor_data)

        self.create_subscription(Image, image_topic,
                                 self.image_callback, qos_profile_sensor_data)


        self.create_subscription(Image, depth_image_topic,
                                 self.depth_image_callback, qos_profile_sensor_data)
        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, 'aruco_markers', 10)

        self.armarker_Info_subscriber_ = self.create_subscription(ArucoMarkers, self.armarker_info_topic, 
                                                   self.armarker_Info_callback, 10)
        image_masking_service = self.create_service(GetMaskImage, "image_masking", 
                                           self.return_mask_image)


        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):

        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg,
                                             desired_encoding='mono8')
        self.cv_image = self.bridge.imgmsg_to_cv2(img_msg,
                                             desired_encoding='bgr8')
        markers = ArucoMarkers()
        pose_array = PoseArray()
        if self.camera_frame is None:
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame
            
            
        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        self.corners, self.marker_ids, rejected = cv2.aruco.detectMarkers(cv_image,
                                                                self.aruco_dictionary,
                                                                parameters=self.aruco_parameters)
        if self.marker_ids is not None:
            self.get_marker = True

            if cv2.__version__ > '4.0.0':
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(self.corners,
                                                                      self.marker_size, self.intrinsic_mat,
                                                                      self.distortion)
            else:
                rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(self.corners,
                                                                   self.marker_size, self.intrinsic_mat,
                                                                   self.distortion)
            
            # for i in range(len(rvecs)):
            #     self.cv_image = cv2.aruco.drawAxis(self.cv_image, 
            #                                        self.intrinsic_mat, 
            #                                        self.distortion, 
            #                                        rvecs[i], 
            #                                        tvecs[i], 
            #                                        0.02)
            # cv2.imshow('QueryImage', self.cv_image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            for i, marker_id in enumerate(self.marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)
        else:
            self.get_marker = False

    def depth_image_callback(self, depth_img_msg):

        self.depth_image = self.bridge.imgmsg_to_cv2(depth_img_msg,
                                             desired_encoding='16UC1')
        
        

    def armarker_Info_callback(self, data):

        # print(self.marker_ids)
        id1_marker_pos = []
        id2_marker_pos = []
        if len(data.marker_ids) >= 8:
            for i in range(8):
                if self.marker_ids[i] == 1:
                    id1_marker_pos.append([int(self.corners[i][0][0][0]), int(self.corners[i][0][0][1])])
                elif self.marker_ids[i] == 2:
                    id2_marker_pos.append([int(self.corners[i][0][0][0]), int(self.corners[i][0][0][1])])
            marker_pos = [id1_marker_pos, id2_marker_pos]
            self.mask_image(marker_pos)
        elif len(data.marker_ids) == 4 and self.marker_ids[:] == 1:

            for i in range(4):
                id1_marker_pos.append([int(self.corners[i][0][0][0]), int(self.corners[i][0][0][1])])
            marker_pos = [id1_marker_pos]

            self.mask_image(marker_pos)
        elif len(data.marker_ids) == 4 and self.marker_ids[:] == 2:

            for i in range(4):
                id2_marker_pos.append([int(self.corners[i][0][0][0]), int(self.corners[i][0][0][1])])
            marker_pos = [id2_marker_pos]

            self.mask_image(marker_pos)
        else:
            self.get_mask_image = False
            return
                
    def mask_image(self,marker_pos):

        masked_img_list = []
        
        print(marker_pos)
        for i in range(len(marker_pos)):
            mask = np.zeros(self.cv_image.shape[:3], np.uint8)
            poly_points = np.array([marker_pos[i][0], marker_pos[i][1], marker_pos[i][2]])
            cv2.fillPoly(mask, pts=[poly_points], color=(255, 255, 255))
            poly_points = np.array([marker_pos[i][0], marker_pos[i][1], marker_pos[i][3]])
            cv2.fillPoly(mask, pts=[poly_points], color=(255, 255, 255))
            poly_points = np.array([marker_pos[i][0], marker_pos[i][2], marker_pos[i][3]])
            cv2.fillPoly(mask, pts=[poly_points], color=(255, 255, 255))

            print(type(self.cv_image[0][0][0]))
            masked_img = np.bitwise_and(self.cv_image, mask)
        
            masked_img_list.append(self.bridge.cv2_to_imgmsg(masked_img, encoding="passthrough"))

        for k in range(len(marker_pos)):
            depth_mask = np.zeros(self.depth_image.shape[:2], np.uint16)
            poly_points = np.array([marker_pos[k][0], marker_pos[k][1], marker_pos[k][2]])
            cv2.fillPoly(depth_mask, pts=[poly_points], color=(255, 255, 255))
            poly_points = np.array([marker_pos[k][0], marker_pos[k][1], marker_pos[k][3]])
            cv2.fillPoly(depth_mask, pts=[poly_points], color=(255, 255, 255))
            poly_points = np.array([marker_pos[k][0], marker_pos[k][2], marker_pos[k][3]])
            cv2.fillPoly(depth_mask, pts=[poly_points], color=(255, 255, 255))
            
            print(type(self.depth_image[0][0]))
            depth_masked_img = np.bitwise_and(self.depth_image, depth_mask)
        
            masked_img_list.append(self.bridge.cv2_to_imgmsg(depth_masked_img, encoding="passthrough"))

        self.masked_image = masked_img_list
        if type(self.masked_image[0]) == type('str'):
            del self.masked_image[0]
        self.get_mask_image = True

    def return_mask_image(self, req, res):
        if not self.get_marker:
            res.image_condition = "bad"
            self.get_mask_image = False
            self.receive_mask = False
            self.masked_image = ['initial']
            return res

        else:
            if self.get_mask_image:
                res.mask_image = self.masked_image
                res.image_condition = "good"
                self.get_mask_image = False
                self.receive_mask = False
                self.masked_image = ['initial']
                return res
            else:
                res.image_condition = "bad"
                self.get_mask_image = False
                self.receive_mask = False
                self.masked_image = ['initial']
                return res
def main():
    rclpy.init()
    node = MaskColorAndDepthImage()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
