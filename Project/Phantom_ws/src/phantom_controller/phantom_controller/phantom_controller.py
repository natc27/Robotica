import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16MultiArray

from dynamixel_sdk import * 

import os, ctypes
import numpy as np

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()

DEFAULT_JOINT_VELOCITY = [25, 25, 25, 25, 25]

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_SPEED      = 38

LEN_MX_GOAL_POSITION       = 2
LEN_MX_MOVING_SPEED        = 2

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_BROADCAST_ID            = 254
DXL_IDS                     = [1, 2, 3, 4, 5]                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM5'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 300           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 400            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

class PhantomController(Node):

    def __init__(self):
        super().__init__("phantom_controller")

        # dynamixel setup
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        
        # Enable Dynamixel Torque

        # for i in range(len(DXL_IDS)):
        #     id = DXL_IDS[i]
        #     self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        #     print('torque enabled in motor ' + str(id))

        # working
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_BROADCAST_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
           print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
           print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
           print("Dynamixel has been successfully connected")

        # variable para escribir informacion en motores
        self.group_num_pos = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
        self.group_num_vel = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED)

        for i in range(len(DXL_IDS)):
            id = DXL_IDS[i]
            goal_vel = DEFAULT_JOINT_VELOCITY[i]
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_MOVING_SPEED, goal_vel)

        self.joint_pos_ = [512, 512, 512, 512, 512]
        self.joint_vel_ = DEFAULT_JOINT_VELOCITY

        # state publishers
        self.joint_pos_pub_ = self.create_publisher(UInt16MultiArray, "/curr_joint_pos", 10)
        self.joint_vel_pub_ = self.create_publisher(UInt16MultiArray, "/curr_joint_vel", 10)

        # goal state subscribers
        self.joint_pos_subscriber_ = self.create_subscription(UInt16MultiArray, "/goal_joint_pos", self.set_goal_joint_pos_callback, 10)
        self.joint_vel_subscriber_ = self.create_subscription(UInt16MultiArray, "/goal_joint_vel", self.set_goal_joint_vel_callback, 10)

        self.timer_ = self.create_timer(0.005, self.send_curr_joint_state)
        self.get_logger().info("Nodo phantom_controller ha iniciado.")  

    def __del__(self):
        # Disable Dynamixel Torque

        # working
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_BROADCAST_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # Close port
        self.portHandler.closePort()

    def set_goal_joint_pos_callback(self, msg: UInt16MultiArray):
        # Write goal position

        # pos_arr = msg.data
        # #print(pos_arr)
        # for i in range(len(DXL_IDS)):
        #     id = DXL_IDS[i]
        #     goal_pos = pos_arr[i]
        #     dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, goal_pos)

        # working!!
        goal_pos_arr = msg.data
        for i in range(len(DXL_IDS)):
            id = DXL_IDS[i]
            goal_pos = goal_pos_arr[i]
            if goal_pos < 1023:
                self.group_num_pos.addParam(id, [goal_pos & 0xFF, (goal_pos >> 8) & 0xFF])

        self.group_num_pos.txPacket()
        self.group_num_pos.clearParam()

    def set_goal_joint_vel_callback(self, msg: UInt16MultiArray):
        # Write goal position

        # goal_vel_arr = msg.data
        # for i in range(len(DXL_IDS)):
        #     id = DXL_IDS[i]
        #     goal_vel = goal_vel_arr[i]
        #     dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_MX_MOVING_SPEED, goal_vel)

        # fijar posición objetivo
        goal_vel_arr = msg.data
        for i in range(len(DXL_IDS)):
            id = DXL_IDS[i]
            goal_vel = goal_vel_arr[i]
            if goal_vel < 1023:
                self.group_num_vel.addParam(id, [goal_vel & 0xFF, (goal_vel >> 8) & 0xFF])

        self.group_num_vel.txPacket()
        self.group_num_vel.clearParam()
                

    def send_curr_joint_state(self):
        msg_pos = UInt16MultiArray()
        msg_vel = UInt16MultiArray()

        # Leer posiciones actuales
        pos_arr = [0, 0, 0, 0, 0]
        for i in range(len(DXL_IDS)):
            id = DXL_IDS[i]
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_POSITION)
            pos_arr[i] = dxl_present_position
        self.joint_pos_ = pos_arr

        # Leer velocidades actuales
        vel_arr = [0, 0, 0, 0, 0]
        for i in range(len(DXL_IDS)):
            id = DXL_IDS[i]
            dxl_present_speed, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_SPEED)
            if dxl_comm_result == COMM_SUCCESS:
                vel_arr[i] = dxl_present_speed
        self.joint_vel_ = vel_arr

        msg_pos.data = pos_arr
        msg_vel.data = vel_arr
        self.joint_pos_pub_.publish(msg_pos)
        self.joint_vel_pub_.publish(msg_vel)

def main(args=None):
    rclpy.init(args=args)

    node = PhantomController()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
