#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Poses
from geometry_msgs.msg import Pose
from numpy import inf

class ArucoPoseFilter(Node):
    def __init__(self, num_cameras):
        super().__init__('aruco_pose_filter')
        self.num_cameras = num_cameras
        self.cameras_of_interest = ['c' + str(i + 0) for i in range(self.num_cameras)]
        self.marker_dict = {}
        self.lastPublish = Pose()
        self.firstPublish = True
        self.camera_subscribers = []

        for camera_name in self.cameras_of_interest:
            topic = camera_name + '/markers'
            self.camera_subscribers.append(self.create_subscription(
                MarkerArray,
                topic,
                lambda msg, camera_name=camera_name: self.marker_callback(msg, camera_name),
                10
            ))

        self.poses_pub = self.create_publisher(Poses, 'aruco_poses', 10)
        """markers = Poses()
        markers.header.frame_id = "origin_frame"
        markers.header.stamp = self.get_clock().now().to_msg()

        firstPose = True
        for marker_id in self.marker_dict.keys():
            pose_filter = Pose()
            min_distance = 100.0
            for camera, info in self.marker_dict[marker_id].items():
                if(firstPose):
                    pose_filter = info[0] 
                    min_distance = info[1]   
                    firstPose = False
                else:
                    if(min_distance > info[1]):
                        pose_filter = info[0] 
                        min_distance = info[1]
                print(camera)
            firstPose = True
            markers.poses.append(pose_filter)
            markers.marker_ids.append(marker_id)
            print(marker_id)
        self.poses_pub.publish(markers)
        """
     

    def marker_callback(self, msg, camera_name):
        for marker in msg.markers:
            pose = marker.pose.pose
            marker_id = marker.id
            marker_distance = marker.distance
            if marker_id in self.marker_dict:
                if camera_name not in self.marker_dict[marker_id]:
                    self.marker_dict[marker_id] = {camera_name: [pose, marker_distance]}
                else:
                    self.marker_dict[marker_id][camera_name] = [pose, marker_distance]
            else:
                self.marker_dict[marker_id] = {camera_name: [pose, marker_distance]}

        if(camera_name == self.cameras_of_interest[-1]):
            markers = Poses()
            markers.header.frame_id = "map"
            markers.header.stamp = self.get_clock().now().to_msg()

            
            for marker_id in self.marker_dict.keys():
                firstPose = True
                pose_filter = Pose()
                min_distance = +inf
                for camera, info in self.marker_dict[marker_id].items():
                    if(firstPose):
                        pose_filter = info[0] 
                        min_distance = info[1]   
                        firstPose = False
                    else:
                        if(min_distance > info[1]):
                            pose_filter = info[0] 
                            min_distance = info[1]
                firstPose = True
                markers.poses.append(pose_filter)
                markers.marker_ids.append(marker_id)
            
            if self.firstPublish:
                self.poses_pub.publish(markers)
                self.firstPublish = False
                self.lastPublish = markers
            else:
                if len(markers.marker_ids) >= len(self.lastPublish.marker_ids):
                    self.poses_pub.publish(markers)
                    self.firstPublish = False
                    self.lastPublish = markers
                else:
                    counter = 0
                    if counter<1000:
                        self.poses_pub.publish(self.lastPublish)
                        counter += 1
                    else:
                        self.poses_pub.publish(markers)
                        self.lastPublish = markers
    
        #markers = Poses()
        #markers.header.frame_id = self.info_msg.header.frame_id
        #markers.header.stamp = msg.header.stamp

        #self.poses_pub.publish(markers)

        

def main(args=None):
    # Pass the number of cameras as a command-line argument
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 subscriber.py <num_cameras>")
        return

    num_cameras = int(sys.argv[1])

    rclpy.init(args=args)
    aruco_subscriber = ArucoPoseFilter(num_cameras)
    rclpy.spin(aruco_subscriber)
    aruco_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
