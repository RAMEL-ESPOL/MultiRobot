import os
from numpy import inf
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Bool
from aruco_msgs.msg import Poses
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from mrs_navigator import MRSNavigator, TaskResult
import math 


class Task(Node):

    def __init__(self, n_robots, id_goal1, id_goal2, id_robot):
        super().__init__('Task'+str(id_robot)+'_'+str(id_goal1)+str(id_goal2))

        self.n_robots = n_robots
        self.id_goal1 = id_goal1
        self.id_goal2 = id_goal2
        self.id_robot = id_robot

        self.subscription = self.create_subscription(
            Poses,
            'aruco_poses',
            self.delivery,
            10)
        self.subscription  # prevent unused variable warning

        


    def delivery(self, msg):

        self.markers_id = msg.marker_ids
        self.poses_array = msg.poses

        marker_index = self.markers_id.index(self.id_goal1)
        self.goal1 = PoseStamped()
        self.goal1.pose.position.x = self.poses_array[marker_index].position.x
        self.goal1.pose.position.y = self.poses_array[marker_index].position.y
        self.goal1.pose.position.z = 0.0
        self.goal1.pose.orientation.x = self.poses_array[marker_index].orientation.x
        self.goal1.pose.orientation.y = self.poses_array[marker_index].orientation.y
        self.goal1.pose.orientation.z = 0.0
        self.goal1.header.stamp = self.get_clock().now().to_msg()
        self.goal1.header.frame_id = "map"

        if not bool(self.id_robot):
            mindistance = +inf
            for id in self.markers_id:
                if(id > 9):
                    marker_index = self.markers_id.index(id)
                    distance_sqrt = (self.goal1.pose.position.x - self.poses_array[marker_index].position.x)**2 + (self.goal1.pose.position.y - self.poses_array[marker_index].position.y)**2  
                    if(distance_sqrt < mindistance):
                        mindistance = distance_sqrt
                        self.id_robot = (id - 9)

        #self.state_pub = self.create_publisher(Bool, '/r'+str(self.id_robot)+'/busy', 10)


        #topic = '/r'+str(self.id_robot)+'/goal_pose'
        #self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/r'+str(self.id_robot)+'/navigate_to_pose')
 
        #print(self.goal1)
        navigator = MRSNavigator(namespace='/r'+str(self.id_robot))
        #navigator.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

        print('Robot' +str(self.id_robot)+': A navegar!!!')
        navigator.goToPose(self.goal1)

        i = 0
        while not navigator.isTaskComplete():
            #self.state_pub.publish(True)
            i += 1

        # Do something depending on the return code
        result1 = navigator.getResult()
        # Update robot's poses
        initial_pose_cmd = "python3 initial_pose_publisher.py "+str(self.n_robots)
        print(initial_pose_cmd)
        os.system(initial_pose_cmd)
        if result1 == TaskResult.SUCCEEDED:
            print('Robot' +str(self.id_robot)+': Goal1 succeeded!')

            marker_index2 = self.markers_id.index(self.id_goal2)
            self.goal2 = PoseStamped()
            self.goal2.pose.position.x = self.poses_array[marker_index2].position.x
            self.goal2.pose.position.y = self.poses_array[marker_index2].position.y
            self.goal2.pose.position.z = 0.0
            self.goal2.pose.orientation.x = self.poses_array[marker_index2].orientation.x
            self.goal2.pose.orientation.y = self.poses_array[marker_index2].orientation.y
            self.goal2.pose.orientation.z = 0.0
            self.goal2.header.stamp = self.get_clock().now().to_msg()
            self.goal2.header.frame_id = "map"

            #print(self.goal2)
            print('Robot' +str(self.id_robot)+': A navegar!!! x2')
            navigator.goToPose(self.goal2)

            i = 0
            while not navigator.isTaskComplete():
                i += 1

            # Update robot's poses
            initial_pose_cmd = "python3 initial_pose_publisher.py 3"
            print(initial_pose_cmd)
            os.system(initial_pose_cmd)

            result2 = navigator.getResult()
            if result2 == TaskResult.SUCCEEDED:
                print('Robot' +str(self.id_robot)+': Goal2 succeeded!')
                self.result = True
            else:
                print('Robot' +str(self.id_robot)+': Goal2 failed!')
                self.result = False
        else:
            print('Robot' +str(self.id_robot)+': Goal1 not completed')
            self.result = False
        
        # Shut down the ROS 2 Navigation Stack
        #navigator.lifecycleShutdown()
        exit(1)


def main(args=None):

    # Pass the arguments
    import sys
    if len(sys.argv) < 4:
        print("Usage: python3 goal_publisher.py <n_robots> <id_goal1> <id_goal1> <id_robot>(optional)")
        return

    n_robots = int(sys.argv[1])
    id_goal1 = int(sys.argv[2])
    id_goal2 = int(sys.argv[3])
    id_robot = 0
    if len(sys.argv) == 5:
        id_robot = int(sys.argv[4])

    rclpy.init(args=args)
    task_manager = Task(n_robots, id_goal1, id_goal2, id_robot)
    rclpy.spin_once(task_manager)
    """
    id_robot = task_managerg1.id_robot
    task_managerg2 = Task(id_goal2, id_robot)

    goal1_result = task_managerg1.result
    print(goal1_result)
    
    if verify_task(task_managerg1.result):
        print('Goal1 complete')

        rclpy.spin_once(task_managerg2)
        print(task_managerg2.result)
        
        if verify_task(task_managerg2.result):
            print('Task complete')
            
        else:
            print('Goal2 no reached')
    else:
        print('Goal1 no reached')

    task_managerg1.destroy_node()
    task_managerg2.destroy_node()
    """
    #future = goal_publisher.goToPose()
    #rclpy.spin_until_future_complete(goal_publisher, future) 
    task_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()