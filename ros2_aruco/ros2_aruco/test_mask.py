import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.srv import GetMaskImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageMaskingClient():

    def __init__(self, node):
        super().__init__()
        self._node = node
        self.cli = self._node.create_client(GetMaskImage, 'image_masking')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info('service not available, waiting again...')
        self.req = GetMaskImage.Request()

    def send_request(self):
        self.req.get_mask = True
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self._node, self.future)
        return self.future.result()


def main():
    rclpy.init(args=None)
    node = rclpy.create_node('image_masking_client')
    masking_client = ImageMaskingClient(node)
    response = masking_client.send_request()

    bridge = CvBridge()
    if "bad" in response.image_condition :
        node.get_logger().info("bad position, cannot get all ar_marker")
    # else:
    elif "good" in response.image_condition:
        cv_image = bridge.imgmsg_to_cv2(response.mask_image[0],
                                     desired_encoding='passthrough')
        cv2.imshow('Masked Image', cv_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        cv_image = bridge.imgmsg_to_cv2(response.mask_image[1],
                                     desired_encoding='passthrough')
        cv2.imshow('Masked Image', cv_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        cv_image = bridge.imgmsg_to_cv2(response.mask_image[2],
                                     desired_encoding='32FC1')
        cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
        cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
        cv2.imshow('Masked Image', cv_image_norm)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        cv_image = bridge.imgmsg_to_cv2(response.mask_image[3],
                                     desired_encoding='32FC1')
        cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
        cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
        cv2.imshow('Masked Image', cv_image_norm)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()