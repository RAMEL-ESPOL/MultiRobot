import os
import subprocess
import asyncio
import concurrent.futures
import threading
from numpy import inf
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from aruco_msgs.msg import Poses#, RobotStates
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from mrs_navigator import MRSNavigator, TaskResult
import math
from navigator_client import Task

class Manager(Node):

    def __init__(self, n_robot):
        super().__init__('Manager')

        self.n_robot = n_robot

        self.subscription = self.create_subscription(
            Poses,
            'aruco_poses',
            self.checkRobots,
            10)
        self.subscription  # prevent unused variable warning

        #self.state_pub = self.create_publisher(RobotStates, '/r'+str(self.id_robot)+'/busy', 10)
    
    def checkRobots(self, msg):
        self.markers_id = msg.marker_ids
        self.poses_array = msg.poses
        # 0 for free and 1 for busy
        self.state_robot = [0]*self.n_robot
        #self.state_pub.publish(self.state_robot)


    def select_robot(self, id_goal1):
        self.id_goal1 = id_goal1
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

        mindistance = +inf
        for id in self.markers_id:
            if(id > 9):
                marker_index = self.markers_id.index(id)
                distance_sqrt = (self.goal1.pose.position.x - self.poses_array[marker_index].position.x)**2 + (self.goal1.pose.position.y - self.poses_array[marker_index].position.y)**2  
                if(distance_sqrt < mindistance and not self.state_robot[id-10]): #and not self.state_robot[id-9]
                    mindistance = distance_sqrt
                    self.id_robot = (id - 9)
        self.state_robot[self.id_robot-1] = 1
        print('States after selection', end="")
        print(self.state_robot)

async def exec_nav(nrobots, id_goal1, id_goal2, id_robot, manager):
    #navigation = ['python3', 'navigator_client.py', str(id_goal1), str(id_goal2), str(id_robot)]
    navigation = 'python3 navigator_client.py '+str(nrobots)+' '+ str(id_goal1)+' '+ str(id_goal2)+' '+ str(id_robot)
    print(navigation)
    process = await asyncio.create_subprocess_shell(navigation)
    #print('Navegation of the robot ' + str(id_robot) + ": ", end="")
    await process.wait()
    manager.state_robot[id_robot-1] = 0
    print(process.returncode)
    print('Estados de los robots despues del viaje:', end="")
    print(manager.state_robot)
    return process.returncode


def exec_task(id_goal1, id_goal2, id_robot):
    navigation = ['python3', 'navigator_client.py', str(id_goal1), str(id_goal2), str(id_robot)]
    #navigation = 'python3 navigator_client.py '+ str(id_goal1)+' '+ str(id_goal2)+' '+ str(id_robot)
    print(navigation)
    process = subprocess.run(navigation)
    return process.returncode

def main(args=None):

    # Pass the arguments
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 goal_publisher.py <n_robots>")
        return
    

    nrobots = int(sys.argv[1])
    while_arg = True
    rclpy.init()
    manager = Manager(nrobots)
    rclpy.spin_once(manager)
    #loop = asyncio.get_running_loop()
    initial_pose_cmd = "python3 initial_pose_publisher.py "+str(nrobots)
    print(initial_pose_cmd)
    os.system(initial_pose_cmd)
    
    while while_arg:
         
        ip = input('Params for navigation: <id_goal1> <id_goal1> <id_robot>(optional).\n')
        print('Params received:', ip, '.')
        params = ip.split()
        goal1 = int(params[0])
        goal2 = int(params[1])
        robot = 0
        print('Estados de los robots antes del viaje:', end="")
        print(manager.state_robot)
        if len(params)>2:
            robot = int(params[2])
        else:
            manager.select_robot(goal1)
            robot = manager.id_robot
        #with concurrent.futures.ProcessPoolExecutor() as pool:
        _thread = threading.Thread(target=asyncio.run, args=(exec_nav(nrobots, goal1, goal2, robot, manager),))
        _thread.start()
        #result = await loop.run_in_executor(pool, exec_task(goal1, goal2, robot))
        #print('Navegation of the robot ' + str(robot) + ": ", end="")
        #print(result)
        #manager.state_robot[robot-1] = 0

        #manager.state_robot[robot] = 0


if __name__ == '__main__':
    asyncio.run(main())