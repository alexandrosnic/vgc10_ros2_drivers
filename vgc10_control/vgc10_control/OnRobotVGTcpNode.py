#!/usr/bin/env python3

import rclpy
import vgc10_modbus_tcp.comModbusTcp
import vgc10_control.baseOnRobotVG
from vgc10_msgs.msg import OnRobotVGInput
from vgc10_msgs.msg import OnRobotVGOutput
import threading


def main():
    try:
        rclpy.init()
        node = rclpy.create_node("OnRobotVGTcpNode")
        node.declare_parameter('/onrobot/ip', '192.168.0.4')
        node.declare_parameter('/onrobot/port', '50002')
        node.declare_parameter('/onrobot/dummy', False)


        ip = node.get_parameter('/onrobot/ip')
        port = node.get_parameter('/onrobot/port')
        dummy = node.get_parameter('/onrobot/dummy')
        
        # Gripper is a VG gripper with a Modbus/TCP connection
        gripper = vgc10_control.baseOnRobotVG.onrobotbaseVG()
        gripper.client = vgc10_modbus_tcp.comModbusTcp.communication(dummy)

        # Connects to the ip address received as an argument
        gripper.client.connectToDevice(ip, port)

        # The Gripper status is published on the topic named 'OnRobotVGInput'
        pub = node.create_publisher(OnRobotVGInput, 'OnRobotVGInput', 1)

        # The Gripper command is received from the topic named 'OnRobotVGOutput'
        subscription = node.create_subscription(OnRobotVGOutput, 'OnRobotVGOutput', gripper.refreshCommand, 10)

        # We loop
        prev_msg = []
        while rclpy.ok():
            # Get and publish the Gripper status
            status = gripper.getStatus()
            pub.publish(status)

            # Spin in a separate thread
            thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
            thread.start()

            rate = node.create_rate(0.05)
            rate.sleep()
            # Send the most recent command
            if not prev_msg == gripper.message:  # find new message
                node.get_logger().info("Sending message.")
                gripper.sendCommand()
            rate.sleep()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()