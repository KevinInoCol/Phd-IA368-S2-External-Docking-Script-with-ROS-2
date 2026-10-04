#!/usr/bin/env python3
"""autodocking.py - external auto-docking node for myRobot (ROS 2, rclpy).

All the logic runs outside CoppeliaSim and talks to it only through the topics of
the remoteAPI_ROS2_bridge launch (ia368_pkg):

  subscribes  /myRobot/charging_base/strengthSignal  std_msgs/Float32   (only published while the IR beam sees the robot)
              /myRobot/charging_base/relativeAngle   std_msgs/Float32   (rad, 0 = dock straight ahead)
              /myRobot/battery_state                 sensor_msgs/BatteryState
              /myRobot/bumper                        geometry_msgs/Wrench (the bumper is at the REAR of myRobot)
              /myRobot/docking_mode                  std_msgs/Int32     (external ON/OFF commands)
  publishes   /myRobot/cmd_vel                       geometry_msgs/Twist
              /myRobot/docking_mode                  std_msgs/Int32

The dock's IR beam is a narrow fan (5 cm wide at the dock, 50 cm at 2 m) in front of the
charging base, so the signal only appears when the robot's docking sensor crosses that corridor.

Behaviour (state machine):
  CLEANING       random-walk cleaning: straight runs, then a short back-up and a random turn
  DOCK_SEARCH    docking ON, no signal: the same random walk until the robot crosses the beam
  DOCK_APPROACH  signal present: turn toward the dock using the relative angle and drive in,
                 slowing down as the signal gets stronger
  DOCK_REACQUIRE signal lost while docking: rotate in place until it is re-acquired
                 (after a full turn without it, back to DOCK_SEARCH)
  CHARGING       docked: stop and wait; when the battery is full or docking is turned OFF -> UNDOCKING
  UNDOCKING      back away from the dock and turn around, then CLEANING (or TELEOP, see 'after_undock')
  TELEOP         publishes nothing, so the joystick UI of the scene drives the robot

Docking is turned ON automatically when the battery is low, or from ROS 2:
  ros2 topic pub --once /myRobot/docking_mode std_msgs/msg/Int32 "{data: 1}"   # 0 turns it OFF

Run (with the bridge already running):
  source /opt/ros/jazzy/setup.bash && python3 autodocking.py
  python3 autodocking.py --ros-args -p low_battery:=60 -p full_battery:=95 -p after_undock:=teleop
"""
import math
import random

import rclpy
from geometry_msgs.msg import Twist, Wrench
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, Int32

# The course bumper_and_velocity_node converts cmd_vel with a 0.0975 m wheel radius and the
# scene controller divides again by its own 0.05 m radius, so the robot moves ~10x faster than
# the commanded Twist. All speeds below are REAL speeds; they are multiplied by this factor.
BRIDGE_SCALE = 0.0975

# real speeds
WALK_SPEED = 0.15         # m/s, cleaning / searching
DOCK_MAX_SPEED = 0.12     # m/s, approach speed far from the dock
DOCK_MIN_SPEED = 0.03     # m/s, approach speed right before docking
TURN_SPEED = 0.8          # rad/s, turns of the random walk and rotate-in-place
K_ANGLE = 1.5             # heading gain toward the dock
ALIGN_ANGLE = 0.3         # rad, larger heading errors: turn in place before driving

# thresholds
BEAM_RANGE = 2.0          # m, range of the IR beam (strength = 1 - d / range)
DOCKED_STRENGTH = 0.955   # the base starts charging when the sensor is <= 0.1 m from the beam
SIGNAL_TIMEOUT = 0.5      # s without strength messages = signal lost
BUMP_FORCE = 1.0          # N, horizontal bumper force considered a collision


class AutoDocking(Node):
    def __init__(self):
        super().__init__('autodocking')
        any_type = ParameterDescriptor(dynamic_typing=True)   # accept 60 as well as 60.0
        self.declare_parameter('low_battery', 60.0, any_type)    # % -> docking ON
        self.declare_parameter('full_battery', 95.0, any_type)   # % -> docking OFF, leave the dock
        self.declare_parameter('after_undock', 'clean')          # 'clean' or 'teleop'
        self.low = float(self.get_parameter('low_battery').value)
        self.full = float(self.get_parameter('full_battery').value)
        self.after_undock = self.get_parameter('after_undock').value

        self.strength = None
        self.angle = 0.0
        self.signal_t = -1e9
        self.battery = None
        self.charging_t = -1e9
        self.bump = False
        self.docking = False

        self.state = None
        self.state_t = 0.0
        self.turned = 0.0
        self.walk = None          # random-walk segment: (phase, end_time, turn_direction)
        self.turn_dir = 1.0       # rotate-in-place direction while re-acquiring the signal
        self.last_step = None

        self.create_subscription(Float32, '/myRobot/charging_base/strengthSignal', self.strength_cb, 10)
        self.create_subscription(Float32, '/myRobot/charging_base/relativeAngle', self.angle_cb, 10)
        self.create_subscription(BatteryState, '/myRobot/battery_state', self.battery_cb, 10)
        self.create_subscription(Wrench, '/myRobot/bumper', self.bumper_cb, 10)
        self.create_subscription(Int32, '/myRobot/docking_mode', self.docking_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/myRobot/cmd_vel', 10)
        self.dock_pub = self.create_publisher(Int32, '/myRobot/docking_mode', 10)

        self.set_state('CLEANING')
        self.create_timer(0.05, self.step)
        self.get_logger().info(f'autodocking started (docking ON below {self.low:.0f} %, OFF at {self.full:.0f} %)')

    # ------------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def strength_cb(self, msg):
        self.strength = msg.data
        self.signal_t = self.now()

    def angle_cb(self, msg):
        self.angle = msg.data

    def battery_cb(self, msg):
        b = msg.percentage
        # the 'Charging' signal is cleared by the battery script, so also trust a rising level
        if msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING or \
                (self.battery is not None and b > self.battery):
            self.charging_t = self.now()
        self.battery = b

    def bumper_cb(self, msg):
        self.bump = math.hypot(msg.force.x, msg.force.y) > BUMP_FORCE

    def docking_cb(self, msg):
        on = msg.data == 1
        if on == self.docking:
            return
        self.docking = on
        self.get_logger().info(f'docking mode {"ON" if on else "OFF"}')
        if on and self.state in ('CLEANING', 'UNDOCKING', 'TELEOP'):
            self.set_state('DOCK_APPROACH' if self.has_signal() else 'DOCK_SEARCH')
        elif not on and self.state in ('DOCK_SEARCH', 'DOCK_APPROACH', 'DOCK_REACQUIRE', 'CHARGING'):
            self.set_state('UNDOCKING')

    def set_docking(self, on):
        self.dock_pub.publish(Int32(data=1 if on else 0))   # docking_node mirrors it in the scene UI
        self.docking_cb(Int32(data=1 if on else 0))

    # ------------------------------------------------------------------ helpers
    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'-> {state}')
        self.state, self.state_t = state, self.now()
        self.turned, self.walk = 0.0, None

    def in_state(self):
        return self.now() - self.state_t

    def has_signal(self):
        return self.strength is not None and self.now() - self.signal_t < SIGNAL_TIMEOUT

    def charging(self):
        return self.now() - self.charging_t < 2.5

    def publish(self, v, w):
        cmd = Twist()
        cmd.linear.x = v * BRIDGE_SCALE
        cmd.angular.z = w * BRIDGE_SCALE
        self.cmd_pub.publish(cmd)

    def random_walk(self):
        """Straight run, short back-up (the rear bumper stops it), bounce turn, repeat.

        Front collisions are not sensed through ROS, so the run is time-limited and the
        turn is large (115-230 deg): a robot pushing against a wall heads back into the room.
        """
        t = self.now()
        if self.walk is None or t > self.walk[1] or (self.walk[0] == 'back' and self.bump):
            phase = 'run' if self.walk is None or self.walk[0] == 'turn' else \
                    'back' if self.walk[0] == 'run' else 'turn'
            duration = {'run': random.uniform(2.5, 5.0), 'back': 0.6,
                        'turn': random.uniform(2.0, 4.0) / TURN_SPEED}[phase]
            self.walk = (phase, t + duration, random.choice((-1.0, 1.0)))
        phase, _, direction = self.walk
        if phase == 'run':
            return WALK_SPEED, 0.0
        if phase == 'back':
            return -WALK_SPEED, 0.0
        return 0.0, direction * TURN_SPEED

    # ------------------------------------------------------------------ main loop
    def step(self):
        t = self.now()
        dt = 0.05 if self.last_step is None else min(t - self.last_step, 0.2)
        self.last_step = t

        if (self.battery is not None and self.battery < self.low and not self.docking
                and self.state in ('CLEANING', 'TELEOP')):
            self.get_logger().info(f'low battery ({self.battery:.0f} %): docking ON')
            self.set_docking(True)

        v = w = 0.0
        if self.state == 'CLEANING':
            v, w = self.random_walk()

        elif self.state == 'DOCK_SEARCH':
            if self.has_signal():
                self.get_logger().info(f'dock signal found (strength {self.strength:.2f})')
                self.set_state('DOCK_APPROACH')     # stop right away: the beam corridor is narrow
            else:
                v, w = self.random_walk()

        elif self.state == 'DOCK_APPROACH':
            if self.strength is not None and self.strength >= DOCKED_STRENGTH or self.charging():
                self.set_state('CHARGING')
            elif not self.has_signal():
                self.get_logger().warn('dock signal lost: rotating in place to re-acquire it')
                self.set_state('DOCK_REACQUIRE')
                self.turn_dir = 1.0 if self.angle >= 0 else -1.0   # toward where the dock was
            else:
                e = math.atan2(math.sin(self.angle), math.cos(self.angle))
                w = max(-TURN_SPEED, min(TURN_SPEED, K_ANGLE * e))
                if abs(e) < ALIGN_ANGLE:
                    dist = (1.0 - self.strength) * BEAM_RANGE
                    v = max(DOCK_MIN_SPEED, min(DOCK_MAX_SPEED, 0.4 * dist)) * math.cos(e)

        elif self.state == 'DOCK_REACQUIRE':
            if self.has_signal():
                self.get_logger().info('dock signal re-acquired')
                self.set_state('DOCK_APPROACH')
            else:
                w = self.turn_dir * TURN_SPEED
                self.turned += abs(w) * dt
                if self.turned > 2.0 * math.pi:            # a full turn without signal
                    self.get_logger().info('no signal after a full turn: searching again')
                    self.set_state('DOCK_SEARCH')

        elif self.state == 'CHARGING':
            if self.battery is not None and self.battery >= self.full:
                self.get_logger().info(f'battery full ({self.battery:.0f} %): docking OFF')
                self.set_docking(False)
            elif self.in_state() > 4.0 and not self.charging():
                # stopped at the dock but not charging: creep in a little more
                if self.has_signal():
                    v = DOCK_MIN_SPEED
                else:
                    self.set_state('DOCK_REACQUIRE')
                    self.turn_dir = 1.0

        elif self.state == 'UNDOCKING':
            if self.in_state() < 1.5 and not self.bump:
                v = -WALK_SPEED                            # back away from the dock (bumper first)
            elif self.in_state() < 1.5 + math.pi / TURN_SPEED:
                w = TURN_SPEED                             # turn around
            else:
                self.set_state('CLEANING' if self.after_undock == 'clean' else 'TELEOP')

        if self.state == 'TELEOP':
            return                                         # leave the robot to the joystick UI
        self.publish(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDocking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
