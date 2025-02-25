#! /usr/bin/env python

import rclpy
import reach_ros_node.driver
from rclpy.node import Node
import glob
import serial
import pyudev
import diagnostic_updater
import diagnostic_msgs


class ros2_ReachSerialHandler(Node):
    # Set our parameters and the default socket to open
    def __init__(self):
        super().__init__('reach_ros_node')
        self.port = ''
        self.emlid = serial.Serial()
        self.device_connected = False
        self.context_ = pyudev.Context()
        self.driver = None
        self.declare_parameters(
            namespace='',
            parameters=[
                ('host', 'reach.local'),
                ('port', 12346),
                ('frame_gps', 'gps'),
                ('frame_timeref', 'gps'),
                ('use_rostime', True),
                ('use_rmc', False)]
        )

    def produce_diagnostics(self, stat):
        # define nominal diagnostic status
        status = diagnostic_msgs.msg.DiagnosticStatus.OK
        summary_msg = 'GPS connected.'

        # check if gps is connected
        if not self.device_connected or self.driver is None:
            status = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            summary_msg = 'GPS not connected'
            stat.add('connected_to_gps', 'false')
            stat.add('gps_has_fix', 'false')
        else:
            stat.add('connected_to_gps', 'true')
            if not self.driver.has_fix:
                summary_msg = 'GPS connected, but has no fix'
                status = diagnostic_msgs.msg.DiagnosticStatus.WARN
                stat.add('gps_has_fix', 'false')
            else:
                stat.add('gps_has_fix', 'true')

        stat.summary(status, summary_msg)
        return stat

    # Should open the connection and connect to the device
    # This will then also start publishing the information
    def start(self):
        # Try to connect to the device
        self.connect_to_device()
        
        if not self.device_connected:
            self.get_logger().info("Could not connect to emlid")
            return

        try:
            self.driver = reach_ros_node.driver.RosNMEADriver(self)
        except Exception as e:
            self.get_logger().error("an error occured while trying to make the driver. Error was: %s." %e)

        try:
            while rclpy.ok():
                data = self.emlid.readline()
                rclpy.spin_once(self, timeout_sec=0) # allow functions such as ros2 node info, ros2 param list to work
                try:
                    self.driver.process_line(data.decode('utf-8').rstrip().encode('utf-8').strip(b'\x00'))
                except ValueError as e:
                    self.get_logger().info("Value error, likely due to missing fields in the NMEA message. Error was: %s." % e)
        except Exception as e:
            self.get_logger().error("an error occured while reading lines from device. Error was: %s." %e)

    # Try to connect to the device, allows for reconnection
    # Will loop till we get a connection, note we have a long timeout
    def connect_to_device(self):
        self.get_logger().info("starting to poll.")
        device_list = self.context_.list_devices(subsystem='tty')
        for device in device_list:
            device_details = dict(device)
            # if device_details['ID_VENDOR'] == 'Emlid':
            if device_details['DEVNAME'][-1] == '0':
                port = str(device_details['DEVNAME'])
                self.get_logger().info("Emlid device found at port = " + port)

                try:
                    self.emlid = serial.Serial(port=port, baudrate=115200)
                    self.device_connected = True
                    break

                except Exception as e:
                    self.get_logger().error("Emlid serial connection attempt failed:")
                    self.get_logger().error(e)


def main(args=None):
    rclpy.init(args=args)

    node = ros2_ReachSerialHandler()
    updater = diagnostic_updater.Updater(node)
    updater.setHardwareID('gps')
    updater.add('diagnostics', node.produce_diagnostics)


    # Start the nodes processing thread
    node.start()

    # at termination of the code (generally with ctrl-c) Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()
