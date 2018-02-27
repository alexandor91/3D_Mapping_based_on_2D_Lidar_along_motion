#! /usr/bin/env python


#     License:BSD
#     This file sweep_fusion.py is to perform scanning continuously and publish the transformed measurements from each scan to the 
#"/sweep_node/cloudpoint" topic to be fused by back-end, here the rotaion is done only for pitch angle
#
#    Maintainer: Alexander.Kang
#
#    Email: alexander.kang@tum.de
#
#    Date: 11.02.2018

"""Defines a 3D scanner"""
import rospy
import argparse
import time
import datetime
import math
import threading
import sweep_constants
import scan_settings
import scan_exporter_f
import scan_utils
import scanner_base
import tf
import std_msgs.msg
import numpy as np
import sensor_msgs.point_cloud2 as pcl2
#import point_cloud_message_creator
from scanner_output import output_json_message
from sweeppy import Sweep
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point 
from sensor_msgs.msg import PointCloud2, PointField

HEADER = Header(frame_id = "map")

FIELDS = [
    # coordinate(x, y, z)
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    # signal strength
    PointField(name='Intensity', offset=9, datatype=PointField.UINT8, count=1),
]
scan_times = 100
base_heading = 0.0
last_heading = 0.0
steady_state_nums  = 0
#valid_scan_index = 0
initial_angle_offset = 0.0
received_angle = False

class Scanner(object):
    """The 3d scanner.
    Attributes:
        base: the rotating base
        device: the sweep scanning LiDAR
        settings: the scan settings
        exporter: the scan exporter
    """

    def __init__(self, device=None, settings=None, exporter=None):
        """Return a Scanner object
        :param base:  the scanner base
        :param device: the sweep device
        :param settings: the scan settings
        :param exporter: the scan exporter
        """
        if device is None:
            self.shutdown()
        if settings is None:
            settings = scan_settings.ScanSettings()
        if exporter is None:
            exporter = scan_exporter.ScanExporter()

        self.device = device
        self.settings = settings
        self.exporter = exporter
        self.received_scan = False
        self.cloud_publisher = rospy.Publisher('/sweep_fusion/cloudpoint', PointCloud2, queue_size = 1) 

    def setup_device(self):
        """Setup the device"""
        reset_max_duration = 11.0

        output_json_message({'type': "update", 'status': "setup",
                             'msg': "Resetting device.", 'duration': reset_max_duration})

        # Reset the device
        self.device.reset()

        # sleep for at least the minimum time required to reset the device
        time.sleep(reset_max_duration)

        output_json_message(
            {'type': "update", 'status': "setup", 'msg': "Adjusting device settings."})

        # Set the sample rate
        self.device.set_sample_rate(self.settings.get_sample_rate())

        # Set the motor speed
        self.device.set_motor_speed(self.settings.get_motor_speed())

    def setup(self):
        """Setup the scanner according to the scan settings"""
        # setup the device, wait for it to calibrate
        self.setup_device()

        # wait until the device is ready, so as not to disrupt the calibration
        while True:
            if self.device.get_motor_ready() is True:
                break

            # Convey that the motor speed is still adjusting
            output_json_message({'type': "update", 'status': "setup",
                                 'msg': "Waiting for calibration routine and motor speed to stabilize."})

            time.sleep(0.5)

    def perform_scan(self):
        """Performs a 3d scan"""
        # Calcualte some intermediate values
        #num_sweeps, angle_between_sweeps, steps_per_move = self.calculate_scan_variables()

        # Report that the scan is initiating, and start scanning
        #self.report_scan_initiated(num_sweeps)
        self.device.start_scanning()

        # put a 3 second timeout on the get_scans() method in case it hangs
        time_out_thread = threading.Timer(5, self.check_get_scan_timeout)
        time_out_thread.start()

        valid_scan_index = 0
        rotated_already = False
        #if not CCW:
        #angle_between_sweeps = -angle_between_sweeps
        # get_scans is coroutine-based generator returning scans ad infinitum
        for scan_count, scan in enumerate(self.device.get_scans()):
            Cloud_Points = []
            # note the arrival time
            scan_arrival_time = time.time()
            HEADER.stamp = rospy.Time.now()
            # note that a scan was received (used to avoid the timeout)
            self.received_scan = True

            # remove readings from unreliable distances
            scan_utils.remove_distance_extremes(
                scan, self.settings.get_min_range_val(), self.settings.get_max_range_val())

            # Remove readings from the deadzone
            scan_utils.remove_angular_window(
                scan, self.settings.get_deadzone(), 360 - self.settings.get_deadzone())

            #if valid_scan_index >= num_sweeps - 2:
                # Avoid redundant data in last few partially overlapping scans

            # Catch scans that contain unordered samples and discard them
            # (this may indicate problem reading sync byte)
            if scan_utils.contains_unordered_samples(scan):
                continue

            # Edge case (discard 1st scan without base movement and move base)
            if not rotated_already:
                # Wait for the device to reach the threshold angle for movement
                self.wait_until_deadzone(scan_arrival_time)

                # Move the base and start again
                #self.base.move_steps(steps_per_move)
                rotated_already = True
                continue

            # Export the scan
            #self.exporter.export_2D_scan(
            #    scan, valid_scan_index, angle_between_sweeps,
            #    self.settings.get_mount_angle(), False)
            #if not CCW:
            #angle_between_sweeps = -angle_between_sweeps
            converted_coords = scan_utils.transform_scan_without_base(scan, self.settings.mount_angle, 0.0)    
            for n, sample in enumerate(scan.samples):
                world_x = converted_coords[n, 0]
                world_y = converted_coords[n, 1] 
                world_z = converted_coords[n, 2]
                self.exporter.writer.writerow({
                    'SCAN_INDEX': valid_scan_index,
                    'X': int(round(world_x)),
                    'Y': int(round(world_y)),
                    'Z': int(round(world_z)),
                    'SIGNAL_STRENGTH': sample.signal_strength
                })
                #print "the samples!"
                row = []
                row.append(float(world_x)/100)
                row.append(float(world_y)/100)
                row.append(float(world_z)/100)
                row.append(sample.signal_strength)
                Cloud_Points.append(row)

            # increment the scan index
            valid_scan_index = valid_scan_index + 1
            print "length of samples is %d" %len(scan.samples)          
            # Wait for the device to reach the threshold angle for movement
            if len(scan.samples) > 0:
               point_cloud2 = pcl2.create_cloud(HEADER, FIELDS, Cloud_Points)       
            #publish the point cloud message via ros
               self.cloud_publisher.publish(point_cloud2)
            # Move the base and report progress
               self.report_scan_progress(valid_scan_index)

            self.wait_until_deadzone(scan_arrival_time)

            # Move the base and report progress
            #self.base.move_steps(steps_per_move)
            self.report_scan_progress(valid_scan_index)

            # Exit after collecting the required number of 2D scans
            #if valid_scan_index >= num_sweeps:
             #   break

        # Stop scanning and report completion
        #time.sleep(1.0)
        #self.device.stop_scanning()
        self.report_scan_complete()
    

    def idle(self):
        """Stops the device from spinning"""
        self.device.set_motor_speed(sweep_constants.MOTOR_SPEED_0_HZ)
     

    def check_get_scan_timeout(self):
        """Checks if we have received a scan from getScan... if not, exit"""
        if not self.received_scan:
            raise ValueError("getScan() never returned... aborting")
            # Should work out a better solution to shutdown.
            # Signaling with KeyboardInterrupt doesn't seem to work and process still hangs
            # Currently the workaround is that the node app will kill this
            # process if it receives an error

    def wait_until_deadzone(self, t_0):
        """ Waits the however long is required to reach the deadzone
        :param t_0: The time the sweep crossed the 0 degree mark
        """
        time_until_deadzone = self.settings.get_time_to_deadzone_sec() - \
            (time.time() - t_0)
        if time_until_deadzone > 0:
            time.sleep(time_until_deadzone)

    def report_scan_initiated(self, num_sweeps):
        """ Reports that a scan has been initiated """
        output_json_message({
          'type': "update",
          'status': "scan",
          'msg': "Initiating scan...",
          'duration': num_sweeps / self.settings.get_motor_speed(),
          'remaining': num_sweeps / self.settings.get_motor_speed()
        })

    def report_scan_progress(self, valid_scan_index):
        """ Reports the progress of a scan """
        rospy.loginfo("the scan order right now is %d with valid points", valid_scan_index) 

    def report_scan_complete(self):
        """ Reports the completion of a scan """
        output_json_message({
            'type': "update",
            'status': "complete",
            'msg': "Finished scan!"
        })

    def shutdown(self):
        """Print message and shutdown"""
        exit()


def main():
    """Creates a 3D scanner and gather a scan"""
    #Initialize Node and handles
    rospy.init_node('sweep_node_motion', anonymous=True)
    exporter = scan_exporter_f.ScanExporter()

    # Create a scanner base
    #base = scanner_base.ScannerBase()

          #settings for sweep
    settings = scan_settings.ScanSettings()
          # Create sweep sensor, and perform scan
    with Sweep('/dev/ttyUSB0') as sweep:
               # Create a scanner object
          time.sleep(1.0)
             
          scanner = Scanner(device=sweep, settings=settings, exporter=exporter)

          # Setup the scanner
          scanner.setup()
          # Perform the scan
          scanner.perform_scan()
          
          while not rospy.is_shutdown():         
             rospy.spin() 
          # Stop the scanner         
               #scanner.idle()
if __name__ == '__main__':
    main()
