from numpy import inf
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from aruco_msgs.msg import Poses
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
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



        #if bool(self.id_robot):
        
        


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
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"

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


        #topic = '/r'+str(self.id_robot)+'/goal_pose'
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/r'+str(self.id_robot)+'/navigate_to_pose')
        
        self.goToPose(goal)
        print(goal)
        #navigator = BasicNavigator()
        #navigator.waitUntilNav2Active(navigator='r'+str(self.id_robot)+'/bt_navigator', localizer='r'+str(self.id_robot)+'/amcl') # if autostarted, else use lifecycleStartup()

        #goal.header.stamp = self.get_clock().now().to_msg()
        #goal.header.frame_id = "map"
        #navigator.goToPose(goal)

        # Do something depending on the return code
        #result = navigator.getResult()
        #if result == TaskResult.SUCCEEDED:
        #    print('Goal succeeded!')
        #elif result == TaskResult.CANCELED:
        #    print('Goal was canceled!')
        #elif result == TaskResult.FAILED:
        #    print('Goal failed!')
        #else:
        #    print('Goal has an invalid return status!')
        
        # Shut down the ROS 2 Navigation Stack
        #navigator.lifecycleShutdown()
        
        #client = self.create_client(NavigateToPose, '/r'+str(self.id_robot)+'/navigate_to_pose')
            
        #print('222222222')
        #future = client.send_goal_async(goal, feedback_callback=None)
        #rclpy.spin_until_future_complete(client, future)
        #future.add_done_callback(self.navigation_result_callback)
        #self.publisher_ = self.create_publisher(
        #    PoseStamped,
        #    topic,
        #    10
        #)
        
        #goal.header.stamp = self.get_clock().now().to_msg()
        #goal.header.frame_id = "map"
        #print(goal)
        #self.publisher_.publish(goal)

    #def navigation_result_callback(future):
        #result = future.result()
        #if result.result == 0:  # 0 indicates success
        #    print("Navigation succeeded!")
        #else:
        #    print("Navigation failed with result code:", result.result)
        #rclpy.shutdown()

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    
    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

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