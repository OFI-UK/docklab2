"""
Node for handling communications with LabJack

Relevant Documentation:
 
LJM Library:
    LJM Library Installer:
        https://labjack.com/support/software/installers/ljm
    LJM Users Guide:
        https://labjack.com/support/software/api/ljm
    Opening and Closing:
        https://labjack.com/support/software/api/ljm/function-reference/opening-and-closing
    eReadName:
        https://labjack.com/support/software/api/ljm/function-reference/ljmereadname
 
T-Series and I/O:
    Modbus Map:
        https://labjack.com/support/software/api/modbus/modbus-map
    Hardware Overview(Device Information Registers):
        https://labjack.com/support/datasheets/t-series/hardware-overview

"""

#!/usr/bin/env python

# Importing ROS2 Python Client Libraries 
import rclpy
from rclpy.node import Node

# Import service type 
from std_srvs.srv import Trigger

# Import message type 
from std_msgs.msg import Float32

# Import Labjack Libraries 
from labjack import ljm

class LabJackNode(Node):
    # Constructor
    def __init__(self):
        # Set node name 
        super().__init__('labjack_node')
        # Set up LaJack
        # Open first found LabJack
        self.handle = ljm.openS("ANY", "ANY", "ANY")  # Any device, Any connection, Any identifier
        #handle = ljm.openS("T7", "ANY", "ANY")  # T7 device, Any connection, Any identifier
        #handle = ljm.openS("T4", "ANY", "ANY")  # T4 device, Any connection, Any identifier
        #handle = ljm.open(ljm.constants.dtANY, ljm.constants.ctANY, "ANY")  # Any device, Any connection, Any identifier
        
        # Log out LabJack info 
        info = ljm.getHandleInfo(handle)
        self.get_logger().info("Opened a LabJack with Device type: %i, Connection type: %i,\n"
              "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" %
              (info[0], info[1], info[2], ljm.numberToIP(info[3]), info[4], info[5]))
        
        # Add read AIN0 publisher
        self.publisher_ = self.create_publisher(Float32, 'topic', 10)
        # Add close device service
        self.srv = self.create_service(Trigger, 'Close', self.Close_callback)

    def eReadName_AIN0_callback(self, request, response):
        # Reading AIN0 from LabJack
        name = "AIN0"
        response.data = ljm.eReadName(self.handle, name)
        # Composing response
        response.success = True
        response.message = 'Read AIN0 Successfully '

        return response
        
    def Close(self, request, response):
        # Closing LabJack device
        ljm.close(self.handle)
        # Composing response
        response.success = True
        response.message = 'Closed LabJack Device Successfully '

        return response


def main():
    rclpy.init()

    LabJack_server = LabJackServer()

    rclpy.spin(LabJack_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
