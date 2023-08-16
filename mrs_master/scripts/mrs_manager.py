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
        super().__init__('Manager_node')

        self.n_robot = n_robot
        # 0 for free and 1 for busy
        self.state_robot = [0]*self.n_robot
        self.markers_id = []
        self.poses_array = []
        self.first_run = True

        initial_pose_cmd = "python3 initial_pose_publisher.py "+str(n_robot)
        print(initial_pose_cmd)
        os.system(initial_pose_cmd)

        self.subscription = self.create_subscription(
            Poses,
            'aruco_poses',
            self.checkRobots,
            10)
        self.subscription  # prevent unused variable warning
     
        #_thread = threading.Thread(target=asyncio.run, args=(self.manager(),))
        #_thread.start()

    
    def checkRobots(self, msg):
        self.markers_id = msg.marker_ids
        self.poses_array = msg.poses
        if self.first_run:
            _thread = threading.Thread(target=asyncio.run, args=(self.manager(),))
            _thread.start()
            self.first_run = False
        
        #print(self.poses_array)
        #self.state_pub.publish(self.state_robot)

    async def manager(self):
        while True:
            if(len(self.markers_id)>0):
                ip = input('Params for navigation: <id_task> <arg1> <arg2> <id_robot>(optional).\n')
                print('Params received:', ip, '.')
                params = ip.split()
                id_task = int(params[0])
                robot = 0
                print('Estados de los robots antes del viaje:', end="")
                print(self.state_robot)
                match id_task:
                    case 1:
                        arg1 = int(params[1])
                        arg2 = int(params[2])
                    case 2:
                        arg1 = float(params[1])
                        arg2 = float(params[2])
                if len(params)>3:
                    if(not self.state_robot[int(params[3])-1]):
                        robot = int(params[3])
                else:
                    match id_task:
                        case 1:
                            self.select_robot_goal(arg1)
                            robot = self.id_robot
                        case 2:
                            self.select_robot_pose(arg1, arg2)
                            robot = self.id_robot
                if robot:
                    self.state_robot[robot-1] = 1
                    print('States after selection', end="")
                    print(self.state_robot)
                    #with concurrent.futures.ProcessPoolExecutor() as pool:
                    _thread = threading.Thread(target=asyncio.run, args=(self.exec_nav(id_task, self.n_robot, arg1, arg2, robot),))
                    _thread.start()
                else:
                    print("No avalible robot")


    def select_robot_goal(self, id_goal1):
        self.id_goal1 = id_goal1
        marker_goal = self.markers_id.index(self.id_goal1)

        self.id_robot = 0
        mindistance = +inf
        for id in self.markers_id:
            if(id > 9):
                marker_robot = self.markers_id.index(id)
                distance_sqrt = (self.poses_array[marker_goal].position.x - self.poses_array[marker_robot].position.x)**2 + (self.poses_array[marker_goal].position.y - self.poses_array[marker_robot].position.y)**2  
                if(distance_sqrt < mindistance and not self.state_robot[id-10]): #and not self.state_robot[id-9]
                    mindistance = distance_sqrt
                    self.id_robot = (id - 9)

    def select_robot_pose(self, x_pose, y_pose):
        self.id_robot = 0
        mindistance = +inf
        for id in self.markers_id:
            if(id > 9):
                marker_robot = self.markers_id.index(id)
                distance_sqrt = (x_pose - self.poses_array[marker_robot].position.x)**2 + (y_pose - self.poses_array[marker_robot].position.y)**2  
                if(distance_sqrt < mindistance and not self.state_robot[id-10]): #and not self.state_robot[id-9]
                    mindistance = distance_sqrt
                    self.id_robot = (id - 9)

    async def exec_nav(self, id_task, nrobots, arg1, arg2, id_robot):
        #navigation = ['python3', 'navigator_client.py', str(id_goal1), str(id_goal2), str(id_robot)]
        navigation = 'python3 navigator_client.py '+str(id_task)+' '+str(nrobots)+' '+ str(arg1)+' '+ str(arg2)+' '+ str(id_robot)
        print(navigation)
        process = await asyncio.create_subprocess_shell(navigation)
        #print('Navegation of the robot ' + str(id_robot) + ": ", end="")
        await process.wait()
        self.state_robot[id_robot-1] = 0
        print(process.returncode)
        print('Estados de los robots despues del viaje:', end="")
        print(self.state_robot)
        return process.returncode
    

def main(args=None):

    # Pass the arguments
    import sys
    if len(sys.argv) < 2:
        print("Usage: python3 goal_publisher.py <n_robots>")
        return
    

    nrobots = int(sys.argv[1])
    rclpy.init()
    manager = Manager(nrobots)
    rclpy.spin(manager)
    manager.destroy_node()
    rclpy.shutdown()
    #loop = asyncio.get_running_loop()
    
    """
    while while_arg:         
        ip = input('Params for navigation: <id_task> <arg1> <arg2> <id_robot>(optional).\n')
        print('Params received:', ip, '.')
        params = ip.split()
        id_task = int(params[0])
        robot = 0
        print('Estados de los robots antes del viaje:', end="")
        print(manager.state_robot)
        if len(params)>3:
            robot = int(params[3])
        else:
            match id_task:
                case 1:
                    arg1 = int(params[1])
                    arg2 = int(params[2])
                    manager.select_robot_goal(arg1)
                    robot = manager.id_robot
                case 2:
                    arg1 = float(params[1])
                    arg2 = float(params[2])
                    manager.select_robot_pose(arg1, arg2)
                    robot = manager.id_robot
        #with concurrent.futures.ProcessPoolExecutor() as pool:
        _thread = threading.Thread(target=asyncio.run, args=(exec_nav(id_task, nrobots, arg1, arg2, robot, manager),))
        _thread.start()
        #result = await loop.run_in_executor(pool, exec_task(goal1, goal2, robot))
        #print('Navegation of the robot ' + str(robot) + ": ", end="")
        #print(result)
        #manager.state_robot[robot-1] = 0

        #manager.state_robot[robot] = 0
    """

if __name__ == '__main__':
    asyncio.run(main())