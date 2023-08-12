import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from aruco_msgs.msg import Poses
from geometry_msgs.msg import PoseWithCovarianceStamped
import math 


class InitialPosePublisher(Node):

    def __init__(self, num_robots, robot_interest):
        super().__init__('initialpose_publisher')

        self.num_robots = num_robots
        self.robot_interest = robot_interest
        self.dataloaded = False
        self.robot_topics = ['r' + str(i + 1) for i in range(self.num_robots)]

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
        """
        if(self.dataloaded):
            print('11111111111')
            for robot_topic in robot_topics:
                
                print('222222222')
                self.publisher_ = self.create_publisher(
                    PoseWithCovarianceStamped,
                    '/'+robot_topic+'/initialpose',
                    10
                )
                msg = PoseWithCovarianceStamped()
                for id in self.markers_id:
                    if((int(robot_topic[-1])-1) == id):
                        msg.pose = self.poses_array[0]
                        break
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "origin_frame"
                print(msg)
                #msg.data = 'Hello World: %d' % self.i
                self.publisher_.publish(msg)
        """

        #self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/'+robot_topic+'/initialpose', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.function_callback)
        #self.i = 0

    def getPoses_callback(self, msg):
        self.markers_id = msg.marker_ids
        self.poses_array = msg.poses
        self.dataloaded = True

        #print('11111111111')
        for robot_topic in self.robot_topics:
            
            #print('222222222')
            self.publisher_ = self.create_publisher(
                PoseWithCovarianceStamped,
                '/'+robot_topic+'/initialpose',
                10
            )
            msg = PoseWithCovarianceStamped()
            for id in self.markers_id:
                if((int(robot_topic[1:])+9) == id):
                    #print(self.poses_array[0])
                    marker_index = self.markers_id.index(id)
                    msg.pose.pose.position.x = self.poses_array[marker_index].position.x
                    msg.pose.pose.position.y = self.poses_array[marker_index].position.y
                    msg.pose.pose.position.z = 0.0
                    msg.pose.pose.orientation.x = 0.0
                    msg.pose.pose.orientation.y = 0.0
                    msg.pose.pose.orientation.z = self.poses_array[marker_index].orientation.z
                    msg.pose.pose.orientation.w = self.poses_array[marker_index].orientation.w
                    msg.pose.covariance[6*0+0] = 0.5 * 0.5
                    msg.pose.covariance[6*1+1] = 0.5 * 0.5
                    msg.pose.covariance[6*5+5] = math.pi/12.0 * math.pi/12.0
                    break
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            if (self.robot_interest == 0):
                print('Initial pose for robot '+str(robot_topic)+' published')
            #msg.data = 'Hello World: %d' % self.i
                self.publisher_.publish(msg)
            else:
                if (self.robot_interest == int(robot_topic[-1])):
                    print('Initial pose for robot '+str(robot_topic)+' published')
                #msg.data = 'Hello World: %d' % self.i
                    self.publisher_.publish(msg)



def main(args=None):

    # Pass the number of robots as a command-line argument
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 subscriber.py <num_robots> <robot_interest>(optional)")
        return

    num_robots = int(sys.argv[1])
    robot_interest = 0
    if len(sys.argv) == 3:
        robot_interest = int(sys.argv[2])

    rclpy.init(args=args)
    initialpose_publiser = InitialPosePublisher(num_robots, robot_interest)
    rclpy.spin_once(initialpose_publiser)
    initialpose_publiser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()