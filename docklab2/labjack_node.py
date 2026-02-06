"""
Node for handling communications with LabJack
"""

#!/user/bin/env python

# Imports
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, String
from labjack import ljm
import time

def labjack_setup(logger):
    handle = ljm.openS("ANY", "ANY", "ANY")
    logger.info("Opened a LabJack device.")

    info = ljm.getHandleInfo(handle)
    logger.info(f"Device type: {info[0]}, Connection type: {info[1]}, Serial number: {info[2]}")
    return handle

def labjack_read(handle, logger):
    aNames = ["AIN0", "AIN1"]
    numFrames = len(aNames)
    aValues = ljm.eReadNames(handle, numFrames, aNames)
    ain0_value = aValues[0]
    ain1_value = aValues[1]
    return {
        "AIN0": ain0_value,
        "AIN1": ain1_value,
    }

def labjack_close(handle, logger):
    ljm.close(handle)
    logger.info("Closed LabJack device.")

def main():
    # Create an instance of Node for the LabJack device
    ljmNode = Node('labjack_node')
    # Use the logger provided by Node
    logger = ljmNode.get_logger()
    # Set up subscription to listen for open/close commands
    ain0_publisher = ljmNode.create_publisher(Float32, 'lbj/ain0', 10)
    ain1_publisher = ljmNode.create_publisher(Float32, 'lbj/ain1', 10)

    handle = labjack_setup(logger)
    try:
        while True:
            values = labjack_read(handle, logger)
            logger.info(f"AIN0: {values['AIN0']}, AIN1: {values['AIN1']}")
            ain0_msg = Float32()
            ain0_msg.data = values["AIN0"]
            ain0_publisher.publish(ain0_msg)
            ain1_msg = Float32()
            ain1_msg.data = values["AIN1"]
            ain1_publisher.publish(ain1_msg)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Test interrupted by user.")
    finally:
        labjack_close(handle, logger)
        rclpy.shutdown()

if __name__ == "__main__":
    rclpy.init()
    main()