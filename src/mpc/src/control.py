#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Twist

from scipy.spatial.transform import Rotation as R 

import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
from robot import Robot
import matplotlib.pyplot as plt

################################################################### Global variables ##################################################################


current_pose =Pose();                                   # Global variable that updates state when required
current_goal = None;
waypoints = [];                                         # Nx3 array [x,y,th]
current_goal = [];
current_waypoint_index = 0;                             # The index of the current waypoint that we are tracking - index of goal
robot_cmd_publisher = 0;
robot_pose_subscriber = 0;

######################################################################### ROS #########################################################################


class PoseSubscriber(Node):                             # Node that houses the subscriber and updates global var for position

    def __init__(self, robot_number = 1):
        super().__init__('pose_subscriber')
        self.num = robot_number
        self.subscription = self.create_subscription(
            Odometry,
            "/robot" + str(robot_number) + "/state",         # The topic to listen to
            self.update_global_state,
            10)
        self.subscription                               # prevent unused variable warning

    def update_global_state(self, msg):
        global current_pose;
        current_pose = msg.pose.pose;                   # Pose contains position(current_pose.position.x/y/z) and orientation (current_pose.orientation.w/x/y/z)
        #self.get_logger().info("pose updated")




class CommandPublisher(Node):

    def __init__(self, robot = 1):
        self.num = robot
        super().__init__('robot_command_publisher')
        self.publisher_ = self.create_publisher(Pose, "robot" + str(robot) + "/cmd_vel", 10)




# This function returns the current state of the robot in [x,y,theta] form. Theta is in Radians I think?
def get_current_state():
    global current_pose;
    curr = current_pose;
    x = curr.position.x;
    y = curr.position.y;
    r = R.from_quat([ curr.orientation.w, curr.orientation.x, curr.orientation.y, curr.orientation.z])
    theta = r.as_euler('zyx')
    #print("Theta is : " ,theta)
    return np.array([x,y,theta[2]]);



#This function checks if the robot's current state is within the radius of the goal, and if so, it changes the goal to the next one. Returns true if the final goal is reached or false otherwise. 
def update_goal(radius=0.01):
    global waypoints;
    global current_waypoint_index;
    global current_goal;

    state = get_current_state();


    if( np.sqrt((current_goal[0]-state[0])**2 + (current_goal[1]-state[1])**2 ) < radius):
        current_waypoint_index = current_waypoint_index+1;
        if(current_waypoint_index >= waypoints.shape[0]):
            print("GOAL REACHED");
            return True;

        current_goal = waypoints[current_waypoint_index,:]
    
    return False


#This function publishes the command U to the robot cmd_publisher refers to
def publish_robot_command (u):
    global robot_cmd_publisher;
    cmd = Twist();
    cmd.linear.x = u[0];
    cmd.angular.z = u[1];
    robot_cmd_publisher.publisher_.publish(cmd);
    print("Command Published   v: ", u[0], "   tdot : ",u[1]);



def update(node):
    rclpy.spin_once(node);


def initialze_ros():
    global robot_pose_subscriber;
    global robot_cmd_publisher;
    global current_pose;

    rclpy.init()

    current_pose = Pose();
    robot_cmd_publisher = CommandPublisher(1);
    robot_pose_subscriber = PoseSubscriber(1);




######################################################################### MPC #########################################################################


def robot_mpc(robot):
  # TODO: Nice comment for function
  
    t0 = 0.0

    dt = 1e-2

    # x = [x0]
    # u = [np.zeros((2,))]
    # t = [t0]

    ur = np.zeros(robot.nu)
    while update_goal():
        # current_time = t[-1]
        current_x = get_current_state() # Only x, y

        xr = current_goal

        current_u_command = robot.compute_mpc_feedback(current_x, xr, ur, dt)

        current_u_real = np.clip(current_u_command, robot.umin, robot.umax)
        # # Autonomous ODE for constant inputs to work with solve_ivp
        # def f(t, x):
        # return robot.continuous_time_full_dynamics(current_x, current_u_real)
        # # Integrate one step
        # sol = solve_ivp(f, (0, dt), current_x, first_step=dt)

        # # Record time, state, and inputs
        # t.append(t[-1] + dt)
        # x.append(sol.y[:, -1])
        # u.append(current_u_command)

        # Publish u
        ur = current_u_real
        publish_robot_command(current_u_real)


    current_u_command = np.zeros(2) # STOP AT GOAL
    print("GOAL REACHED")
    return True

    #   x = np.array(x)
    #   u = np.array(u)
    #   t = np.array(t)
    #   return x, u, t

def main(args):

    initialze_ros()
    global waypoints;
    

    number_of_robots = args[1]
    print("Number of robots: ", number_of_robots)
    R = np.diag(5, 10);
    Q = np.diag([10, 10, 0.1]);
    Qf = Q;

    robot = robot(Q, R, Qf);

    # TODO: get map
    
    # Initial state
    start = [(0, 0)]
    goal  = [(5, 5)]

    # TODO: Call astar service
    waypoints = np.array([  [0, 0]
                            [1, 0]
                            [2, 1]
                            [3, 2]
                            [4, 1]
                            [5, 0]
                            [6, 1]
                            [6, 2]
                            [5, 3]
                            [5, 4]
                            [5, 5]])

    robot_mpc(robot)



if __name__ == '__main__':
    main()
