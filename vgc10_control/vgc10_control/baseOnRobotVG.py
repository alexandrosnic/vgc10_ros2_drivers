#!/usr/bin/env python3

import rclpy
from vgc10_msgs.msg import OnRobotVGInput


class onrobotbaseVG():
    """Base class (communication protocol agnostic) for sending commands
       and receiving the status of the OnRobot VG gripper.
    """

    def __init__(self):
        # Initiate output message as an empty list
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verifyCommand(self, command):
        """Verifies that the value of each variable satisfy its limits."""

        # Verify that each variable is in its correct range
        command.rvca = max(0, command.rvca)
        command.rvca = min(255, command.rvca)
        command.rvcb = max(0, command.rvcb)
        command.rvcb = min(255, command.rvcb)

        # Verify that the selected mode number is available
        if command.rmca not in [0x0000, 0x0100, 0x0200]:
            rclpy.signal_shutdown(
                "Select the mode number for ch A from" +
                "0x0000 (release), 0x0100 (grip), or 0x0200 (idle).")
        if command.rmcb not in [0x0000, 0x0100, 0x0200]:
            rclpy.signal_shutdown(
                "Select the mode number for ch B from" +
                "0x0000 (release), 0x0100 (grip), or 0x0200 (idle).")

        # Return the modified command
        return command

    def refreshCommand(self, command):
        """Updates the command which will be sent
           during the next sendCommand() call.
        """

        # Limit the value of each variable
        command = self.verifyCommand(command)

        # Initiate command as an empty list
        self.message = []

        # Build the command with each output variable
        self.message.append(command.rmca)
        self.message.append(command.rvca)
        self.message.append(command.rmcb)
        self.message.append(command.rvcb)

    def sendCommand(self):
        """Sends the command to the Gripper."""

        self.client.sendCommand(self.message)

    def getStatus(self):
        """Requests the status from the gripper and
           return it in the OnRobotVGInput msg type.
        """

        # Acquire status from the Gripper
        status = self.client.getStatus()

        # Message to output
        message = OnRobotVGInput()

        # Assign the values to their respective variables
        message.gvca = status[0]
        message.gvcb = status[1]

        return message
