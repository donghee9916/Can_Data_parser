#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import can
from custom_msgs.msg import AdAccCmd, AdBrkmotCmd, AdAccInfo, AdBrkmotInfo


class CANPublisher(Node):
    def __init__(self):
        super().__init__('can_publisher')

        # ROS topic -> CAN
        self.ad_brkmot_sub = self.create_subscription(AdBrkmotCmd, 'ad_brkmot_cmd',self.ad_brkmot_callback, 10)
        self.ad_acc_sub = self.create_subscription(AdAccCmd, 'ad_acc_cmd',self.ad_acc_callback, 10)

        # CAN -> ROS topic
        self.ad_brkmot_pub = self.create_publisher(AdBrkmotInfo, 'ad_brkmot_info', 10)
        self.ad_acc_pub = self.create_publisher(AdAccInfo, 'ad_acc_info', 10)

        # Initialize CAN bus
        self.can_bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

        # Timer callback Setting
        timer_period = 0.01 # seconds, 10ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    
    def ad_brkmot_callback(self, msg):
        ad_brkmot_alive_cnt = msg.adbrkmotalivecnt
        ad_brkmot_chksum = msg.adbrkmotchksum
        ad_brkmot_ctrlmode = msg.adbrkmotctrlmode
        ad_brkmot_cmd_pos = msg.adbrkmotcmd_pos
        ad_brkmot_cmd_vel = msg.adbrkmotcmd_vel
        ad_brkmot_takeover = msg.adbrkmottakeover

        # CAN 메시지로 변환
        can_msg_data = bytearray()
        can_msg_data.extend(ad_brkmot_alive_cnt.to_bytes(1, byteorder='big'))  # 8비트 데이터
        can_msg_data.extend(ad_brkmot_chksum.to_bytes(1, byteorder='big'))  # 8비트 데이터
        can_msg_data.extend(ad_brkmot_ctrlmode.to_bytes(1, byteorder='big'))  # 8비트 데이터
        can_msg_data.extend(ad_brkmot_cmd_pos.to_bytes(1, byteorder='big'))  # 8비트 데이터
        can_msg_data.extend(ad_brkmot_cmd_vel.to_bytes(2, byteorder='big'))  # 16비트 데이터
        can_msg_data.extend(ad_brkmot_takeover.to_bytes(1, byteorder='big'))  # 8비트 데이터

        # CAN 메시지 전송
        can_msg = can.Message(arbitration_id=0x300, data=can_msg_data)
        self.can_bus.send(can_msg)
    
    # Callback function for receiving AD_ACC_INFO message from CAN bus
    def ad_acc_callback(self, msg):
        # AD_ACC_CMD 메시지로부터 데이터 추출
        ad_acc_alive_cnt = msg.adaccalivecnt
        ad_acc_chksum = msg.adaccchksum
        ad_acc_ctrlmode = msg.adaccctrlmode
        ad_acc_cmd_pos = msg.adacccmd_pos
        ad_acc_cmd_vel = msg.adacccmd_vel
        ad_acc_takeover = msg.adacctakeover

        # CAN 메시지로 변환
        can_msg_data = bytearray()
        can_msg_data.extend(ad_acc_alive_cnt.to_bytes(1, byteorder='big'))  # 8비트 데이터
        can_msg_data.extend(ad_acc_chksum.to_bytes(1, byteorder='big'))  # 8비트 데이터
        can_msg_data.extend(ad_acc_ctrlmode.to_bytes(1, byteorder='big'))  # 8비트 데이터
        can_msg_data.extend(ad_acc_cmd_pos.to_bytes(1, byteorder='big'))  # 8비트 데이터
        can_msg_data.extend(ad_acc_cmd_vel.to_bytes(2, byteorder='big'))  # 16비트 데이터
        can_msg_data.extend(ad_acc_takeover.to_bytes(1, byteorder='big'))  # 8비트 데이터

        # CAN 메시지 전송
        can_msg = can.Message(arbitration_id=0x310, data=can_msg_data)  # arbitration_id 수정 필요
        self.can_bus.send(can_msg)

    # Publish AD_BRKMOT_CMD message to CAN bus
    def publish_ad_brkmot_cmd(self, msg):
        pub_msg = AdBrkmotInfo()
        pub_msg.trzbrkmotalivecnt = msg.data[0]
        pub_msg.trzbrkmotchksum = msg.data[1]
        pub_msg.trzbrkmotctrlmodeinfo = msg.data[2]
        pub_msg.trzbrkmotcmdposinfo = msg.data[3]
        pub_msg.trzbrkmotcmdvelinfo = (msg.data[4] << 8) | msg.data[5]  # 16비트 데이터
        pub_msg.trzbrkmottakeoverinfo = msg.data[6]
        pub_msg.trzbrkmotstatusinfo = msg.data[7]
        self.ad_brkmot_pub.publish(pub_msg)

    # Publish AD_ACC_CMD message to CAN bus
    def publish_ad_acc_cmd(self, msg):
        pub_msg = AdAccInfo()
        pub_msg.trzaccalivecnt = msg.data[0]
        pub_msg.trzaccchksum = msg.data[1]
        pub_msg.trzaccctrl_modinfo = msg.data[2]
        pub_msg.trzacccmdpos_info = msg.data[3]
        pub_msg.trzacctakeoverinfo = msg.data[4]
        pub_msg.trzaccstatusinfo = (msg.data[5] << 8) | msg.data[6]
        self.ad_acc_pub.publish(pub_msg)

    def timer_callback(self):
        while True:
            message = self.can_bus.recv() 
            if message.arbitration_id == 0x600:
                self.publish_ad_brkmot_cmd(message)
            elif message.arbitration_id == 0x610:
                self.publish_ad_acc_cmd(message)


def main(args=None):
    rclpy.init(args=args)
    Can_parser = CANPublisher()

    rclpy.spin(Can_parser)

    Can_parser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()