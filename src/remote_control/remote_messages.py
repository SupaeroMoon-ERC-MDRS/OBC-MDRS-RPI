## To create a ROS node to process incoming messages from the remote control
"""Untested code for now"""
from udpcanpy import NetworkHandler, RemoteControl #to access the UPDCAN protocol
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class RemoteComms(Node):

    def __init__(self,protocol:str = "/home/davidgmolnar/Documents/COM-2024/COM-2024-DBC/comms.dbc"): # string is placeholder to be replaced with actual path on device
        super().__init__("RemoteComms") #placeholder name for now
        """TO ADD: Logger initialisation and parameter declaration (if needed)"""

        ## Create subscriptions and publishers
        self.cmd_vel_pub = self.create_publisher(msg_type=Twist,topic="/cmd_vel",qos_profile=10)
        self.telem_sub = self.create_subscription(msg_type=Float64,topic="/telemetry",qos_profile=10)
        """The queue size has been set to 10 for now, but it can be changed as necessary"""
        
        ## Next step is to create interface with comms protocol to get remote input and store it for the node to publish
        
        # first creating NetworkHandler object
        self.nh = NetworkHandler()
        # then reading the UDPCAN rules from the file
        self.res = self.nh.parse(protocol)
        
        if self.res != 0:
            print("Failed to parse UDPCAN protocol, error code:", self.res)
            """Do we need some sort of exit clause here for the error? - ask Em or En when they are freer"""

        self.res = self.nh.init()
        if self.res != 0:
            print("nh Failed to init, error code:", self.res)
            
        
        self.res = self.nh.start()
        if self.res != 0:
            print("Failed to start thread, error code:", self.res)
        

        self.remote = self.nh.getRemoteControl() #This creates the higher order structure that contains the data we need to access - MessageWrapper equivalent, I think
        self.data = RemoteControl() #object to eventually store the message data when accessed
        
        self.e_stop = False #creating emergency stop attribute so that we know when that's been pressed

        """Q to ask: When writing in ROS, does everything that would otherwise be a regular variable, now become an attribute? - yes, for now"""
    
        ##initialise speed variables - follow Emma's keyboard controls py file as template for updating and packaging as Twist
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        
        # values by which to increment the speeds when a button is pressed
        self.lin_inc = 0.1
        self.ang_inc = 0.2
        
        # setting max limits
        self.max_lin_speed = 5.0
        self.max_ang_speed = 3.0
        """Q for Emma - do I create a timer? - yes"""
        # Timer to run remote input method repeatedly once the Node is initialised
        self.timer = self.create_timer(0.5,self.remote_input)

    def remote_input(self):
       # method to access the input message from the remote control and publish to topic
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
                self.L2 = self.data.l_trigger # int 0-255 # L2 on PS4, LT on Xbox
                self.R2 = self.data.r_trigger # int 0-255 # R2 on PS4, RT on Xbox
                self.ThumbLX: self.data.thumb_left_x # int 0-255
                self.ThumbLY: self.data.thumb_left_y # int 0-255
                self.ThumbRX: self.data.thumb_right_x # int 0-255 
                self.ThumbRY: self.data.thumb_right_y # int 0-255

                # now update velocities based on new inputs
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

                if self.RB: #normal stop button - values reset to zero before creating and publishing Twist
                    self.lin_speed = 0.0
                    self.ang_speed = 0.0
                
                # Twist message to store and send the current command values
                rov_cmd = Twist()
                rov_cmd.linear.x = self.lin_speed
                rov_cmd.angular.z = self.ang_speed
                self.cmd_vel_pub.publish(rov_cmd)
                
    def emergency_stop(self):
        """Base code for this method, to be developed further"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        """Next steps to add: Toggle switch, Robotic Arm message definition, Robotic arm remote input,
        and integration of rover and robotic arm controls"""
        """To be added later: proper error handling and emergency stop procedure"""

