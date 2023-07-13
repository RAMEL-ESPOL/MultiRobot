import rclpy
import rospy
from rclpy.node import Node

from std_msgs.msg import String
from aruco_msgs.msg import Poses
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPosePublisher(Node):

    def __init__(self, num_robots):
        super().__init__('initialpose_publisher')

        self.num_robots = num_robots
        robot_topics = ['r' + str(i + 1) for i in range(self.num_robots)]

        self.subscription = self.create_subscription(
            Poses,
            'aruco_poses',
            self.getPoses_callback,
            10)
        self.subscription  # prevent unused variable warning

        #for robot_topic in robot_topics:
        #    self.create_subscription(
        #        Poses,
        #        'aruco_poses',
        #        lambda msg, robot_topic=robot_topic: self.marker_callback(msg, robot_topic),
        #        10
        #    )

        for robot_topic in robot_topics:
            self.publisher_ = self.create_publisher(
                PoseWithCovarianceStamped,
                '/'+robot_topic+'/initialpose',
                lambda msg, robot_topic=robot_topic: self.initialPoses_callback(msg, robot_topic),
                10
            )

        #self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/'+robot_topic+'/initialpose', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.function_callback)
        #self.i = 0

    def getPoses_callback(self, msg):
        self.markers_id = msg.marker_ids
        self.poses_array = msg.poses
        print()


    def initialPoses_callback(self, msg, robot_topic):

        for id in self.markers_id:
            if((int(robot_topic[-1])-1) == id):
                msg.pose = self.poses_array[0]
                break
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "map"

        #msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1


def main(args=None):

    # Pass the number of cameras as a command-line argument
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 subscriber.py <num_robots>")
        return

    num_robots = int(sys.argv[1])

    rclpy.init(args=args)
    initialpose_publiser = InitialPosePublisher(num_robots)
    rclpy.spin(initialpose_publiser)
    initialpose_publiser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()