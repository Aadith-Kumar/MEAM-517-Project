#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Twist

from scipy.spatial.transform import Rotation as R 


current_pose =Pose();                                   # Global variable that updates state when required
current_goal = None;
waypoints = [];                                         # Nx3 array [x,y,th]
x_goal = [];
current_waypoint_index = 0;                             # The index of the current waypoint that we are tracking - index of goal
robot1_cmd_publisher = 0;
robot1_pose_subscriber = 0;



class PoseSubscriber(Node):                             # Node that houses the subscriber and updates global var for position

    def __init__(self, robot_number = 1):
        super().__init__('pose_subscriber')
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
        super().__init__('robot_command_publisher')
        self.publisher_ = self.create_publisher(Pose, "robot" + str(robot) + "/cmd_vel", 10)






# This function returns the current state of the robot in [x,y,theta] form. Theta is in Radians I think?
def get_current_state():
    global current_pose;
    x = current_pose.position.x;
    y = current_pose.position.y;
    r = R.from_quat([ current_pose.orientation.w, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z])
    theta = r.as_euler('zyx')
    #print("Theta is : " ,theta)
    return np.array([x,y,theta[2]]);



#This function checks if the robot's current state is within the radius of the goal, and if so, it changes the goal to the next one. Returns true if the final goal is reached or false otherwise. 
def update_goal(radius=0.01):
    global waypoints;
    global current_waypoint_index;
    global x_goal;

    state = get_current_state();


    if( np.sqrt((x_goal[0]-state[0])**2 + (x_goal[1]-state[1])**2 ) < radius):
        current_waypoint_index = current_waypoint_index+1;
        if(current_waypoint_index >= waypoints.shape[0]):
            print("GOAL REACHED");
            return True;

        x_goal = waypoints[current_waypoint_index,:]
    
    return False

#This function publishes the command U to the robot cmd_publisher refers to
def publish_robot_command(cmd_publisher, u):
    cmd = Twist();
    cmd.linear.x = u[0];
    cmd.angular.z = u[1];
    cmd_publisher.publisher_.publish(cmd);
    self.get_logger().info("Command Published   v: ", u[0], "   tdot : ",u[1]);



def update(node):
    rclpy.spin_once(node);



def main(args=None):

    global robot1_pose_subscriber;
    global robot1_cmd_publisher;
    rclpy.init(args=args)
    
    robot1_cmd_publisher = CommandPublisher()
    robot1_pose_subscriber = PoseSubscriber()

    while True:
        update(robot1_pose_subscriber);
        x = get_current_state()

        print("theta is : ", x[2])


    robot1_cmd_publisher.destroy_node()
    robot1_pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()