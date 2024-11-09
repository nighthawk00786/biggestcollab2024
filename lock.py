import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
prev_az=0.0

class move_stick(Node):

    def __init__(self):
        super().__init__('move_stick')
        self.prev_az = 10 #can be any value except 0
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(Joy,'/joy',self.listener_callback,10)
        self.subscriber = self.create_subscription(Odometry,'/odom',self.bno_callback,10)
        self.lockin = False
        self.currentangle = 0.0
        self.currentangle = msg.pose.pose.orientation.z

    def listener_callback(self,msg:Joy):
        vel = Twist()
        global prev_az
        #finding global vx and vy from joy
        self.global_vx = float(msg.axes[1])
        self.global_vy = float(msg.axes[0])
        #converting global vx and vy to local using BNO output
        vel.linear.x =  self.global_vx*math.cos(self.currentangle) + self.global_vy*math.sin(self.currentangle)
        vel.linear.y = -self.global_vx*math.sin(self.currentangle) + self.global_vy*math.cos(self.currentangle)

        #finding global az from joy
        self.current_az = float(msg.axes[3])
        if prev_az==0 and msg.buttons[0]==1:
                self.lockin = not self.lockin
        if self.current_az!=0:
            self.target_angle = self.currentangle
        if self.current_az==0:
            if self.lockin:
                self.basket()
                self.target_angle = self.b_angle
            # if self.current_az == 0:#Calling PID if angular velocity is not given
            self.PID()
        prev_az = float(msg.buttons[0])
        
        #Sending out angular velocity and updating prev angular velocity to current angular velocity
        vel.angular.z = self.current_az
        self.prev_az = self.current_az
        self.publisher_.publish(vel)

    def PID(self):
        error = -self.currentangle + self.target_angle
        k = 1 #PID P Value
        if abs(error)>0.0987 and abs(error)<6.1844:
            self.current_az = k*error
    
    def basket(self):
        x2 = 5.0
        y2 = 5.0
        del_x = x2-self.msg.pose.pose.position.x
        del_y = y2-self.msg.pose.pose.position.y
        if del_x==0:
            if del_y>0:
                self.b_angle = 1.57079
            else:
                self.b_angle = 4.71238
        else:
            self.b_angle = math.atan(del_y/del_x)
        if del_x>0:
        
            if (del_y<0):
            
                self.b_angle = 360-abs(self.b_angle)
            
        
        elif (del_x<0):
        
            self.b_angle = 180+self.b_angle
        
        






def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = move_stick()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()