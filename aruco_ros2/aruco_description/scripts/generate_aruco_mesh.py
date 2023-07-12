#!/usr/bin/env python3

# import the necessary packages
import rclpy
import rclpy.node
import numpy as np
import cv2
import sys
import os
from ament_index_python.packages import get_package_share_directory

# Get package path
#ROSPACK = rospkg.RosPack()
#PKG_PATH = ROSPACK.get_path('aruco_description')
#DEBUG_PLOT = False
PKG_PATH = get_package_share_directory('aruco_description')
# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class ArucoGenerator(rclpy.node.Node):
    
    def __init__(self):
        super().__init__('aruco_generator_node')

        self.declare_parameter('aruco_dictionary', "DICT_4X4_50")
        self.declare_parameter('aruco_ids', '0')

        self.timer = self.create_timer(1, self.generate_aruco)
    
    def generate_dae(self, png_filename, out_path, dae_template):
        data = {"pngFilename": png_filename}
        new_dae = dae_template.format(**data)
        # write the dae template
        with open(out_path, mode='w') as file:
            file.write(new_dae)


    def generate_tag(self, arucoDict, tag_id, out_path, img_path):
        tag = np.zeros((300, 300, 1), dtype="uint8")
        cv2.aruco.drawMarker(arucoDict, tag_id, 300, tag, 1)
        #cv2.aruco.generateImageMarker(arucoDict, tag_id, 300, tag, 1)

        # write the generated ArUCo tag to disk and then display it
        cv2.imwrite(out_path, tag)
        cv2.imwrite(img_path, tag)
        cv2.imshow("ArUCo Tag", tag)
        cv2.waitKey(100)


    def generate_aruco(self):
        #rclpy.init()

        # Get params
        #node = rclpy.create_node('aruco_generator')
        aruco_ids = self.get_parameter('aruco_ids').get_parameter_value().string_value
        if isinstance(aruco_ids, str):
            # If multiple elements split by a comma are provided
            aruco_ids = aruco_ids.split(',')
            aruco_ids = [int(id) for id in aruco_ids]
        else:
            # if only one number is provided
            aruco_ids = int(aruco_ids)
            if aruco_ids == -1:
                aruco_ids = []
            else:
                aruco_ids = [aruco_ids]
        
        dict_type = self.get_parameter('aruco_dictionary').get_parameter_value().string_value

        # Check inputs
        if ARUCO_DICT.get(dict_type, None) is None:
            self.get_logger().error("ArUCo tag of '{}' is not supported".format(dict_type))
            return False
        max_aruco_id = int(dict_type.split('_')[-1])
        aruco_ids.sort()

        if len(aruco_ids) == 0:
            aruco_ids = [i for i in range(max_aruco_id)]
        else:
            if not aruco_ids[-1] < max_aruco_id:
                self.get_logger().error("The max id for {} is {}. Requested id was {} ".format(
                    dict_type, max_aruco_id, aruco_ids[-1]))
                return False

        # load the ArUCo dictionary
        arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dict_type])
        output_folder = os.path.join(PKG_PATH, 'meshes', dict_type)
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        # load the dae template
        with open(os.path.join(PKG_PATH, 'model', 'template.dae'), mode='r') as file:
            dae_template = file.read()

        output_imgs_folder = os.path.join(PKG_PATH, 'images', dict_type)
        if not os.path.exists(output_imgs_folder):
            os.makedirs(output_imgs_folder)
            
        for aruco_id in aruco_ids:
            self.get_logger().info("generating ArUCo tag type '{}' with ID '{}'".format(dict_type, aruco_id))
            img_filename = "Marker{}.png".format(aruco_id)
            png_filename = "{}_id{}.png".format(dict_type, aruco_id)
            dae_filename = "{}_id{}.dae".format(dict_type, aruco_id)
            png_path = os.path.join(output_folder, png_filename)
            img_path = os.path.join(output_imgs_folder, img_filename)
            dae_path = os.path.join(output_folder, dae_filename)
            self.generate_tag(arucoDict, aruco_id, png_path, img_path)
            self.generate_dae(png_filename, dae_path, dae_template)

        return True

def main():
    rclpy.init()
    node = ArucoGenerator()
    try:
        if not node.generate_aruco():
            sys.exit(-1)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()