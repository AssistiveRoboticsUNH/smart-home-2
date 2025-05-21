'''
    This node implements the IR docking algorithm for the SHR robot.
    It uses the IR sensor to orient the robot towards the docking station
    and the LIDAR to get docking station distance.
    The robot will stop when it is bumped or when it is charging.

    Rotation control is done using a PID controller.

    author: @mnakash
'''

import time
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int64
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Docking_IR(Node):

    def __init__(self):
        super().__init__('docking_ir')

        # TF listener (keep it alive)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # --- state flags ---
        self.bumped = False
        self.bump = None
        self.is_charging = False
        self.voltage = None
        self.charging_current = None
        self.ir_sensor_weight = None
        self.sensor_data_receiving = False
        self.isLidarActive = False
        self.mode = 'far'
        self.close_counter = 0
        self.sensor_oriented = False # Check if sensor is oriented to docking station for the first time

        # --- class‐level tuning parameters ---
        # IR‐ring setpoint and deadband
        self.ir_setpoint    = 4.0      # “ideal” average index
        self.ir_deadband    = 0.10     # ± this is “close enough”
        # PID gains
        self.kp             = 0.6
        self.ki             = 0.1
        self.kd             = 0.05
        # PID internal state
        self._pid_integral  = 0.0
        self._prev_error    = 0.0
        self._last_time     = time.time()
        self.max_angular    = 0.16     # clamp on angular.z

        # forward speeds for modes
        self.forward_speed_far   = -0.05
        self.forward_speed_close = -0.025
        self.forward_speed       = self.forward_speed_far

        # obstacle‐back threshold logic
        self.obstacle_back = float('inf')

        # --- subscriptions ---
        self.create_subscription(Int64,      'bump',                self.bump_callback,             10)
        self.create_subscription(Float32,    'charging_voltage',    self.charging_voltage_callback,10)
        self.create_subscription(Float32,    'charging_current',    self.charging_current_callback,10)
        self.create_subscription(Float32,    'docking/ir_weight',   self.ir_sensor_callback,       10)
        self.create_subscription(LaserScan,  '/scan_filtered',      self.scan_callback,            10)

        # --- publisher ---
        cmd_topic = os.getenv("cmd_vel", "/cmd_vel")
        self.pub = self.create_publisher(Twist, cmd_topic, 1)


    def bump_callback(self, msg: Int64):
        self.bump = msg.data

    def charging_voltage_callback(self, msg: Float32):
        self.voltage = msg.data

    def charging_current_callback(self, msg: Float32):
        prev = self.charging_current
        self.charging_current = msg.data

        # detect connect / disconnect events
        if prev is not None:
            if prev < 0 <= self.charging_current:
                self.get_logger().info("Charger connected.")
                self.is_charging = True
            elif prev >= 0 > self.charging_current:
                self.get_logger().info("Charger disconnected.")
                self.is_charging = False

    def ir_sensor_callback(self, msg: Float32):
        self.ir_sensor_weight = msg.data

        if self.ir_sensor_weight == -1:
            self.sensor_data_receiving = False
        else:
            self.sensor_data_receiving = True

    def scan_callback(self, msg: LaserScan):
        # Scans laser and returns obstacle distane towards the back of the robot.
        # look at the 4/8 to 4.5/8 arc in the back
        start_ind = int(4 * (len(msg.ranges) / 8))  # 0
        end_ind = int(4.5 * (len(msg.ranges) / 8) - 1)

        truncated_ranges = msg.ranges[start_ind:end_ind]
        # print("tracated", truncated_ranges)
        self.obstacle_back = min(msg.ranges[start_ind:end_ind])
        self.isLidarActive = True
        # print("obstacle_back: ", self.obstacle_back)

        ## Set the mode, whethre robot is close to docking station or far away
        if 0 < self.obstacle_back < 0.6:
            self.close_counter += 1
            if self.close_counter > 15:
                if self.obstacle_back < 0.48: # if very close proximity detected
                    self.mode = 'very_close'
                else:
                    self.mode = 'close'
        elif self.obstacle_back > 0.65: # if enough amount of close proximity not detected consider station not close (handles null laser values)
            self.close_counter = 0
            self.mode = 'far'


    def move_robot(self, lin_x: float, ang_z: float):
        t = Twist()
        t.linear.x  = lin_x
        t.angular.z = ang_z
        self.pub.publish(t)


    def move_to_docking_station(self) -> int:
        # 1) wait for bump info
        if self.bump is None:
            self.get_logger().info("Waiting for bump sensor...")
            time.sleep(0.5)
            return 1

        # 2) ensure LIDAR is alive
        if not self.isLidarActive: self.get_logger().info(f'Lidar: Not Active')

        if self.isLidarActive:
            # 3) if already bumped or charging → stop docking
            if self.bump or self.is_charging:
                self.bumped = True
                # reset PID & orientation flag
                self._pid_integral = 0.0
                self._prev_error   = 0.0
                self.sensor_data_receiving = False
                self.sensor_oriented = False
                print("Bumped / Charging, docking done.")
                return 0

            # 4) do we have IR data?
            if not self.sensor_data_receiving or self.ir_sensor_weight is None:
                self.get_logger().info("Waiting for IR data...")
                return 1

            # 5) PID on IR error
            now   = time.time()
            dt    = now - self._last_time if now > self._last_time else 0.01
            self._last_time = now

            error = self.ir_sensor_weight - self.ir_setpoint

            # within deadband → zero output + reset integral
            if abs(error) <= self.ir_deadband:
                ang_vel = 0.0
                self._pid_integral = 0.0
                self.sensor_oriented = True
            else:
                # integral with simple windup guard
                self._pid_integral += error * dt
                self._pid_integral = max(min(self._pid_integral, self.max_angular), -self.max_angular)

                derivative = (error - self._prev_error) / dt
                raw_out   = self.kp * error + self.ki * self._pid_integral + self.kd * derivative
                ang_vel   = -raw_out   # invert so +error → turn back toward setpoint
                ang_vel   = max(min(ang_vel, self.max_angular), -self.max_angular)
                self._prev_error = error


            # 6) choose forward and angular velocity based on proximity mode
            if self.mode == 'close' or self.mode == 'very_close':
                self.forward_speed = self.forward_speed_close
                if self.mode == 'very_close':
                    self.get_logger().info("Very close to docking station")
                    self.ang_vel = 0.0
            else:
                self.forward_speed = self.forward_speed_far

            # 7) if sensor is not oriented to docking station, rotate only, no forward speed
            if not self.sensor_oriented:
                self.get_logger().info("Rotating to orient IR sensor...")
                self.forward_speed = 0.0

            # 8) drive
            self.move_robot(self.forward_speed, ang_vel)
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = Docking_IR()
    # keep calling move_to_docking_station until bumped
    while rclpy.ok() and not node.bumped:
        node.move_to_docking_station()
        rclpy.spin_once(node)
    # final status
    print("Charger Connected" if node.is_charging else "Charger Not Connected")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
