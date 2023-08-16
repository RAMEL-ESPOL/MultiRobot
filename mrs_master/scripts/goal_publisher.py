from numpy import inf
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from aruco_msgs.msg import Poses
from geometry_msgs.msg import PoseStamped
import math 


class GoalPublisher(Node):

    def __init__(self, id_goal, id_robot):
        super().__init__('goalpose_publisher')

        self.id_goal = id_goal
        self.id_robot = id_robot

        self.subscription = self.create_subscription(
            Poses,
            'aruco_poses',
            self.getPoses_callback,
            10)
        self.subscription  # prevent unused variable warning



    def getPoses_callback(self, msg):

        self.markers_id = msg.marker_ids
        self.poses_array = msg.poses
        topic = ""

        marker_index = self.markers_id.index(self.id_goal)
        goal = PoseStamped()
        goal.pose.position.x = self.poses_array[marker_index].position.x
        goal.pose.position.y = self.poses_array[marker_index].position.y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = self.poses_array[marker_index].orientation.x
        goal.pose.orientation.y = self.poses_array[marker_index].orientation.y
        goal.pose.orientation.z = 0.0
        

        if not bool(self.id_robot):
        #    topic = '/r'+self.id_robot+'/goal_pose'
        #else:
            
            mindistance = +inf
            for id in self.markers_id:
                if(id > 9):
                    marker_index = self.markers_id.index(id)
                    distance_sqrt = (goal.pose.position.x - self.poses_array[marker_index].position.x)**2 + (goal.pose.position.y - self.poses_array[marker_index].position.y)**2  
                    if(distance_sqrt < mindistance):
                        mindistance = distance_sqrt
                        self.id_robot = (id - 9)


        topic = '/r'+str(self.id_robot)+'/goal_pose'
            
        print('222222222')
        self.publisher_ = self.create_publisher(
            PoseStamped,
            topic,
            10
        )
        
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        print(goal)
        self.publisher_.publish(goal)


def main(args=None):

    # Pass the arguments
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 goal_publisher.py <id_goal> <id_robot>(optional)")
        return

    id_goal = int(sys.argv[1])
    id_robot = 0
    if len(sys.argv) == 3:
        id_robot = int(sys.argv[2])

    rclpy.init(args=args)
    goal_publisher = GoalPublisher(id_goal, id_robot)
    rclpy.spin_once(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()