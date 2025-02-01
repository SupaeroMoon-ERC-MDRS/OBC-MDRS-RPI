## To create a ROS node to process incoming messages from the remote control
"""Untested code for now"""
from udpcanpy import NetworkHandler, RemoteControl #to access the UPDCAN protocol
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist, Quaternion


class RemoteComms(Node):

    def __init__(self,protocol:str = "src/sim_mdrs/sim_mdrs/comms.dbc"): # string is placeholder to be replaced with actual path on device
        super().__init__("RemoteComms") #placeholder name for now
        """TO ADD: Logger initialisation and parameter declaration (if needed)"""

        ## Create subscriptions and publishers
        self.cmd_vel_pub = self.create_publisher(msg_type=Twist,topic="/cmd_vel",qos_profile=10)
        self.cmd_arm_motion_pub = self.create_publisher(msg_type=Quaternion,topic="/cmd_move_arm",qos_profile=10)
        self.cmd_arm_grip_pub = self.create_publisher(msg_type=Bool,topic="/cmd_grip_arm",qos_profile=10)
        # self.telem_sub = self.create_subscription(msg_type=Float64,topic="/telemetry",qos_profile=10)
        """The queue size has been set to 10 for now, but it can be changed as necessary"""
        
        ## Next step is to create interface with comms protocol to get remote input and store it for the node to publish
        
        # first creating NetworkHandler object
        self.nh = NetworkHandler()
        # then reading the UDPCAN rules from the file
        self.res = self.nh.parse(protocol)
        
        if self.res != 0:
            self.get_logger().error(f"Failed to parse UDPCAN protocol, error code: {self.res}")
            """Do we need some sort of exit clause here for the error? - ask Em or En when they are freer"""

        self.res = self.nh.init()
        if self.res != 0:
            self.get_logger().error(f"nh Failed to init, error code: {self.res}")
            
        
        self.res = self.nh.start()
        if self.res != 0:
            self.get_logger().error(f"Failed to start thread, error code: {self.res}")
        

        self.remote = self.nh.getRemoteControl() #This creates the higher order structure that contains the data we need to access - MessageWrapper equivalent, I think
        self.data = RemoteControl() #object to eventually store the message data when accessed
        
        self.e_stop = False #creating emergency stop attribute so that we know when that's been pressed

        """Q to ask: When writing in ROS, does everything that would otherwise be a regular variable, now become an attribute? - yes, for now"""
        ## Toggle to switch between different modes
        self.arm_mode = False #rover mode by default

        ##initialise rover control variables - follow Emma's keyboard controls py file as template for updating and packaging as Twist
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        
        # values by which to increment the speeds when a button is pressed
        self.lin_inc = 0.1
        self.ang_inc = 0.2

        # setting max limits
        self.max_lin_speed = 5.0
        self.max_ang_speed = 3.0

        ##initialise arm control variables
        #initial motion directions
        self.arm_x = 0.0 #forward/back
        self.arm_z = 0.0 #up/down
        self.arm_theta = 0.0 #arm swivel
        self.end_theta = 0.0 #gripper (end-vector) rotation
        self.end_grip = False #gripper grab or not

        """Q for Emma - do I create a timer? - yes"""
        # Timer to run remote input method repeatedly once the Node is initialised
        self.timer = self.create_timer(0.1,self.remote_input)

    def remote_input(self):
       ## main method to access the input message from the remote control and publish to topic
        if self.e_stop:
            self.emergency_stop()
        elif not self.e_stop:
            self.res = self.remote.access(self.data) #accesses message within remote and puts it into the data object (RemoteControl object)
            if self.res == 0: #no error code
                self.e_stop = self.data.e_stop # bool # overwriting emergency stop variable with actual input
                self.LB = self.data.l_bottom # bool
                self.LT = self.data.l_top # bool
                self.LR = self.data.l_right # bool
                self.LL = self.data.l_left # bool
                self.RB = self.data.r_bottom # bool
                self.RT = self.data.r_top # bool
                self.RR = self.data.r_right # bool
                self.RL = self.data.r_left # bool
                self.L1 = self.data.l_shoulder # bool # L1 on PS4, LS/LB on Xbox
                self.R1 = self.data.r_shoulder # bool # R1 on PS4, RS/RB on Xbox
                self.L2 = self.data.left_trigger # int 0-255 # L2 on PS4, LT on Xbox
                self.R2 = self.data.right_trigger # int 0-255 # R2 on PS4, RT on Xbox
                self.ThumbLX = self.data.thumb_left_x # int 0-255
                self.ThumbLY = self.data.thumb_left_y # int 0-255
                self.ThumbRX = self.data.thumb_right_x # int 0-255 
                self.ThumbRY = self.data.thumb_right_y # int 0-255

                #self.get_logger().error(self)

                self.get_logger().error(f"Arm mode? {self.arm_mode}")

                if self.L1 and self.R1:
                    self.arm_mode = not self.arm_mode

                if not self.arm_mode:
                    self.rover_command()
                elif self.arm_mode:
                    self.arm_command()

    def __repr__(self):
        return (f"=================\n\
                    LB: {self.LB}\n\
                    LT: {self.LT}\n\
                    LR: {self.LR}\n\
                    LL: {self.LL}\n\
                    RB: {self.RB}\n\
                    RT: {self.RT}\n\
                    RR: {self.RR}\n\
                    RL: {self.RL}\n\
                    LS: {self.L1}\n\
                    RS: {self.R1}\n\
                    LTrigger:{self.L2}\n\
                    RTrigger:{self.R2}\n\
                    ThumbLX: {self.ThumbLX}\n\
                    ThumbLY: {self.ThumbLY}\n\
                    ThumbRX: {self.ThumbRX}\n\
                    ThumbRY: {self.ThumbRY}\n")
    
    def print_remote_data(self):
        self.get_logger().debug(f"=================\n\
                    LB: {self.data.l_bottom}\n\
                    LT: {self.data.l_top}\n\
                    LR: {self.data.l_right}\n\
                    LL: {self.data.l_left}\n\
                    RB: {self.data.r_bottom}\n\
                    RT: {self.data.r_top}\n\
                    RR: {self.data.r_right}\n\
                    RL: {self.data.r_left}\n\
                    LS: {self.data.l_shoulder}\n\
                    RS: {self.data.r_shoulder}\n\
                    LTrigger: {self.data.left_trigger}\n\
                    RTrigger: {self.data.right_trigger}\n\
                    ThumbLX: {self.data.thumb_left_x}\n\
                    ThumbLY: {self.data.thumb_left_y}\n\
                    ThumbRX: {self.data.thumb_right_x}\n\
                    ThumbRY: {self.data.thumb_right_y}\n\
                        ")
                
    def rover_command(self):
        ## method to update and publish velocity commands in rover mode
        # update velocities based on new inputs
        if self.LT:
            self.lin_speed += self.lin_inc
        elif self.LB:
            self.lin_speed -= self.lin_inc
        elif self.LL:
            self.ang_speed += self.ang_inc
        elif self.LR:
            self.ang_speed -= self.ang_inc

        # Clamp the speeds to their maximum values
        self.lin_speed = max(min(self.lin_speed, self.max_lin_speed), -self.max_lin_speed)
        self.ang_speed = max(min(self.ang_speed, self.max_ang_speed), -self.max_ang_speed)

        #print to debug
        self.get_logger().debug(f"speeds calculated to send: {self.lin_speed,self.ang_speed}")
        #print to debug
        self.get_logger().debug(f"speeds being sent: {self.lin_speed,self.ang_speed}")
        if self.RB: #normal stop button - values reset to zero before creating and publishing Twist
            self.lin_speed = 0.0
            self.ang_speed = 0.0
        
        # Twist message to store and send the current command values
        rov_cmd = Twist()
        rov_cmd.linear.x = self.lin_speed
        rov_cmd.angular.z = self.ang_speed
        self.cmd_vel_pub.publish(rov_cmd)

    def arm_command(self):
 
        self.arm_x = 0.0 #forward/back
        self.arm_z = 0.0 #up/down
        self.arm_theta = 0.0 #arm swivel
        self.end_theta = 0.0 #gripper (end-vector) rotation
        self.end_grip = False #gripper grab or not

        # Left analog stick up/down for forward/back
        if self.ThumbLY <= 5:
            self.arm_x = 1.0
        elif self.ThumbLY >= 250:
            self.arm_x = -1.0
        # Right analog stick up/down for up/down
        if self.ThumbRY <= 5:
            self.arm_z = 1.0
        elif self.ThumbRY >= 250:
            self.arm_z = -1.0
        # Left analog stick right/left for rotation
        if self.ThumbLX <= 5: #left
            self.arm_theta = 1.0 #positive usually means CCW by RH rule
        elif self.ThumbLX >= 250: #right
            self.arm_theta = -1.0
        # up/down direction buttons on the left for wrist rotation
        if self.LT:
            self.end_theta = 1.0
        elif self.LB:
            self.end_theta = -1.0
        # triangle button (right top) to grab
        if self.RT:
            self.end_grip = True

        #print to debug
        self.get_logger().debug(f"arm commands to send: {self.arm_x, self.arm_z}")
        # #stop button
        if self.RB:
            self.arm_x = 0.0
            self.arm_z = 0.0 
            self.arm_theta = 0.0 
            self.end_theta = 0.0 
            self.end_grip = False


        #print to debug
        self.get_logger().error(f"arm commands being sent: {self.arm_x,self.arm_z}")

        arm_cmd = Quaternion()
        arm_cmd.x = self.arm_x #x is forward/back
        arm_cmd.z = self.arm_z #z is up/down
        arm_cmd.y = self.arm_theta #y is rotation
        arm_cmd.w = self.end_theta #w is wrist/end-vector joint position
        self.cmd_arm_motion_pub.publish(arm_cmd)
        end_cmd = Bool()
        end_cmd.data = self.end_grip
        self.cmd_arm_grip_pub.publish(end_cmd)

    
    def emergency_stop(self):
        """Base code for this method, to be developed further"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        movearm = Quaternion()
        self.cmd_arm_motion_pub.publish(movearm)
        end_cmd = Bool()
        end_cmd.data = False
        self.cmd_arm_grip_pub.publish(end_cmd)
        raise Exception("EMERGENCY! Stopping...")

def main(args=None):
    rclpy.init(args=args)
    node = RemoteComms()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"{e}")
    finally:
        node.nh.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()