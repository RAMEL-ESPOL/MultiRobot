import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose

class ArucoSubscriber(Node):
    def __init__(self, num_cameras):
        super().__init__('aruco_subscriber')
        self.num_cameras = num_cameras
        self.cameras_of_interest = ['c' + str(i + 0) for i in range(self.num_cameras)]
        self.marker_dict = {}

        for camera_name in self.cameras_of_interest:
            topic = camera_name + '/markers'
            self.create_subscription(
                MarkerArray,
                topic,
                lambda msg, camera_name=camera_name: self.marker_callback(msg, camera_name),
                10
            )

    def marker_callback(self, msg, camera_name):
        for marker in msg.markers:
            pose = marker.pose.pose
            marker_id = marker.id
            if marker_id in self.marker_dict:
                if camera_name not in self.marker_dict[marker_id]:
                    self.marker_dict[marker_id][camera_name] = pose
            else:
                self.marker_dict[marker_id] = {camera_name: pose}
        print(self.marker_dict)

def main(args=None):
    # Pass the number of cameras as a command-line argument
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 subscriber.py <num_cameras>")
        return

    num_cameras = int(sys.argv[1])

    rclpy.init(args=args)
    aruco_subscriber = ArucoSubscriber(num_cameras)
    rclpy.spin(aruco_subscriber)
    aruco_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
