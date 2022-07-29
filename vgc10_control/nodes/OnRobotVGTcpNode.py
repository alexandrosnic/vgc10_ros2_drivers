#!/usr/bin/env python3

import rospy
import onrobot_vg_modbus_tcp.comModbusTcp
import onrobot_vg_control.baseOnRobotVG
from vgc10_msgs.msg import OnRobotVGInput
from vgc10_msgs.msg import OnRobotVGOutput




if __name__ == '__main__':
    try:
        rclpy.init()
        node = rclpy.create_node("OnRobotVGTcpNode")

        ip = node.get_parameter('/onrobot/ip', '192.168.1.1')
        port = node.get_parameter('/onrobot/port', '502')
        dummy = node.get_parameter('/onrobot/dummy', False)
        
        # Gripper is a VG gripper with a Modbus/TCP connection
        gripper = onrobot_vg_control.baseOnRobotVG.onrobotbaseVG()
        gripper.client = onrobot_vg_modbus_tcp.comModbusTcp.communication(dummy)

        # Connects to the ip address received as an argument
        gripper.client.connectToDevice(ip, port)

        # The Gripper status is published on the topic named 'OnRobotVGInput'
        pub = node.create_publisher(OnRobotVGInput, 'OnRobotVGInput', queue_size=1)

        # The Gripper command is received from the topic named 'OnRobotVGOutput'
        subscription = node.create_subscription(OnRobotVGOutput, 'OnRobotVGOutput', gripper.refreshCommand)

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