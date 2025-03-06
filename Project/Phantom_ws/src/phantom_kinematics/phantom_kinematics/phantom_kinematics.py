import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, UInt16MultiArray, Float32MultiArray, Bool

DEFAULT_JOINT_VEL_VEC = [25, 25, 25, 25, 25]
MIN_JOINT_VEL = [5, 5, 5, 5, 5]
MAX_JOINT_VEL = [25, 25, 25, 25]
DEFAULT_JOINT_POS_VEC = [512, 512, 512, 512, 512]
JOYSTICK_THRESHOLD = 0.5

class PhantomKinematics(Node):

    def __init__(self):
        super().__init__("phantom_kinematics")

        self.joystick_axes_ = [0, 0, 0, 0, 0, 0]
        self.joint_pos_ = DEFAULT_JOINT_POS_VEC
        self.r1_button_down_ = False
        self.x_button_down_ = False # 1
        self.circ_button_down_ = False # 2
        self.sqr_button_down_ = False # 3

        self.goal_joint_pos_pub_ = self.create_publisher(UInt16MultiArray, "/goal_joint_pos", 10)
        self.goal_joint_vel_pub_ = self.create_publisher(UInt16MultiArray, "/goal_joint_vel", 10)

        self.r1_button_down_subscriber_ = self.create_subscription(Bool, "/r1_button", self.r1_button_down_callback, 10)
        self.joystick_axes_subscriber_ = self.create_subscription(Float32MultiArray, "filtered_axes", self.joystick_axes_callback, 10)
        self.joint_pos_subscriber_ = self.create_subscription(UInt16MultiArray, "/curr_joint_pos", self.joint_pos_callback, 10)
        self.joystick_buttons_subscriber_ = self.create_subscription(Int32MultiArray, "/filtered_buttons", self.joystick_buttons_callback, 10)

        self.vel_timer_ = self.create_timer(0.005, self.send_goal_joint_vel)
        self.pos_timer_ = self.create_timer(0.005, self.send_goal_joint_pos)

        self.get_logger().info("Nodo phantom_kinematics creado.")


    def r1_button_down_callback(self, msg: Bool):
        self.r1_button_down_ = msg.data

    def joystick_buttons_callback(self, msg: Int32MultiArray):
        self.x_button_down_ = bool(msg.data[0])
        self.circ_button_down_ = bool(msg.data[1])
        self.sqr_button_down_ = bool(msg.data[2])


    def joystick_axes_callback(self, msg: Float32MultiArray):
        self.joystick_axes_ = msg.data

    def joint_pos_callback(self, msg: UInt16MultiArray):
        self.joint_pos_ = msg.data

    def send_goal_joint_vel(self):
        msg = UInt16MultiArray()
        goal_vel = DEFAULT_JOINT_VEL_VEC

        if self.r1_button_down_:
            if self.joystick_axes_[0] > JOYSTICK_THRESHOLD or self.joystick_axes_[0] < -JOYSTICK_THRESHOLD:
                goal_vel[0] = int(MIN_JOINT_VEL[0] + abs((MAX_JOINT_VEL[0] - MIN_JOINT_VEL[0])*self.joystick_axes_[0]))
            else:
                goal_vel[0] = 1023 # out of range, does nothing

            if self.joystick_axes_[1] > JOYSTICK_THRESHOLD or self.joystick_axes_[1] < -JOYSTICK_THRESHOLD:
                goal_vel[2] = int(MIN_JOINT_VEL[1] + abs((MAX_JOINT_VEL[1] - MIN_JOINT_VEL[1])*self.joystick_axes_[1]))
            else:
                goal_vel[2] = 1023 # out of range, does nothing
            
            if self.joystick_axes_[4] > JOYSTICK_THRESHOLD or self.joystick_axes_[4] < -JOYSTICK_THRESHOLD:
                goal_vel[2] = int(MIN_JOINT_VEL[2] + abs((MAX_JOINT_VEL[2] - MIN_JOINT_VEL[2])*self.joystick_axes_[4]))
            else:
                goal_vel[2] = 1023 # out of range, does nothing

            if self.joystick_axes_[2] > JOYSTICK_THRESHOLD or self.joystick_axes_[2] < -JOYSTICK_THRESHOLD:
                goal_vel[3] = int(MIN_JOINT_VEL[3] + abs((MAX_JOINT_VEL[3] - MIN_JOINT_VEL[3])*self.joystick_axes_[2]))
            else:
                goal_vel[3] = 1023 # out of range, does nothing
            
            msg.data = [goal_vel[0], goal_vel[1], goal_vel[2], goal_vel[3], goal_vel[4]]
            self.goal_joint_vel_pub_.publish(msg)
        else:
            msg.data = [1023, 1023, 1023, 1023, 1023]
            self.goal_joint_vel_pub_.publish(msg) # do nothing

    def send_goal_joint_pos(self):
        msg = UInt16MultiArray()
        goal_pos = self.joint_pos_

        if self.r1_button_down_:
            if self.joystick_axes_[0] > JOYSTICK_THRESHOLD:
                goal_pos[0] = max(0, min(1022, goal_pos[0] + 2))
            elif self.joystick_axes_[0] < -JOYSTICK_THRESHOLD:
                goal_pos[0] = max(0, min(1022, goal_pos[0] - 2))
            else:
                goal_pos[0] = 1023 # out of range, does nothing

            if self.joystick_axes_[1] > JOYSTICK_THRESHOLD:
                goal_pos[1] = max(0, min(1022, goal_pos[1] + 2))
            elif self.joystick_axes_[1] < -JOYSTICK_THRESHOLD:
                goal_pos[1] = max(0, min(1022, goal_pos[1] - 2))
            else:
                goal_pos[1] = 1023
            
            if self.joystick_axes_[4] > JOYSTICK_THRESHOLD:
                goal_pos[2] = max(0, min(1022, goal_pos[2] + 2))
            elif self.joystick_axes_[4] < -JOYSTICK_THRESHOLD:
                goal_pos[2] = max(0, min(1022, goal_pos[2] - 2))
            else:
                goal_pos[2] = 1023

            if self.joystick_axes_[2] > JOYSTICK_THRESHOLD:
                #print('1')
                goal_pos[3] = max(0, min(1022, goal_pos[3] + 2))
            elif self.joystick_axes_[2] < -JOYSTICK_THRESHOLD:
                #print('2')
                goal_pos[3] = max(0, min(1022, goal_pos[3] - 2))
            else:
                goal_pos[3] = 1023

            if self.x_button_down_:
                goal_pos[4] = max(0, min(1022, goal_pos[4] + 2))
            elif self.circ_button_down_:
                goal_pos[4] = max(0, min(1022, goal_pos[4] - 2))
            else:
                goal_pos[4] = 1023
            
            msg.data = [goal_pos[0], goal_pos[1], goal_pos[2], goal_pos[3], goal_pos[4]]
            #print(self.joystick_axes_[2])
            #print(msg.data)
            self.goal_joint_pos_pub_.publish(msg)
        else:
            msg.data = [1023, 1023, 1023, 1023, 1023]
            self.goal_joint_pos_pub_.publish(msg) # do nothing

        print(msg.data)
        
def main(args=None):
    rclpy.init(args=args)
    node = PhantomKinematics()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()