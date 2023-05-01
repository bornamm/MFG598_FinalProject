import rclpy
import numpy
from rclpy.node import Node
from robdep_interfaces.msg import BlobList, BlobData
from geometry_msgs.msg import Twist

# Define calibration parameters for tuning the controller
# PID Gains
Angular_Kp = 0.9
Angular_Ki = 0.105
Angular_Kd = 0.2
Linear_Kp = 0.9
Linear_Ki = 0.105
Linear_Kd = 0.2
# Sample time Cal
dt = 0.1
# PID Set points, used for error checking and tuning
BobAreaSP = 3500
CenterSP = 120
# PID output saturation for velocity command
MinVel = -0.025
MaxVel = 0.025
MinVel_Ang = -0.1
MaxVel_Ang = 0.1

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, MinVel, MaxVel):
        # set up cals
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        # buffer
        self.integral = 0
        self.previous_error = 0
        # saturation
        self.MaxVel = MaxVel
        self.MinVel = MinVel

    # Saturation function to limit the output velocity
    def sturation(self, x, min_val, max_val):
        y = numpy.clip(x, min_val, max_val)
        return y
    
    def update(self, process_variable, dt):
        error = self.setpoint - process_variable
        # proportional term
        proportional = self.Kp * error
        # integral term
        self.integral += error * dt
        integral = self.Ki * self.integral
        # derivative term
        derivative = self.Kd * (error - self.previous_error) / dt
        self.previous_error = error
        # output
        outputUnSaturated = proportional + integral + derivative
        output = self.sturation(outputUnSaturated, self.MinVel, self.MaxVel)
        return output

#initialize the controllers for ANgular and linear velocity commands with respective gains
Linear_controller = PIDController(Angular_Kp, Angular_Ki, Angular_Kd, BobAreaSP, MinVel, MaxVel)
Angular_controller = PIDController(Linear_Kp, Linear_Ki, Linear_Kd, CenterSP, MinVel_Ang, MaxVel_Ang)

class BlobPrinter(Node):
    def __init__(self):
        super().__init__('blob_printer')
        # set up the subscription to the Blob data, Tx from blob_detector Rx by blob_subscription
        self.subscription = self.create_subscription(BlobList, '/blobs_list', self.listener_callback, 1)
        self.subscription  # prevent unused variable warning
        self.twist = Twist()
        # set up publisher node for the velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize controller inputs
        self.c_size = 0
        self.cy = 0
        self.cx = 0

    # Read message data and assign to the controller inputs
    def listener_callback(self, msg):
        blob_data = msg.data[0]
        self.cx = blob_data.x_point # read x pos
        self.cy = blob_data.y_point # read y pos
        self.c_size = blob_data.size # read size of the area

    def timer_callback(self):
        # Linear velocity commands
        self.twist.linear.x = Linear_controller.update(self.c_size, dt) # run PID for current time step
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        # Angular velocity commands
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = Angular_controller.update(self.cx, dt) # run PID for current time step
        # publish velcity commands
        self.publisher_.publish(self.twist)
        
# Main algorithm loop
def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = BlobPrinter()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

# Entery point to the algorithm
if __name__ == '__main__':
    main() # Main algorithm loop function call


