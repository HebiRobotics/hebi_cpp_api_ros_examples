#!/usr/bin/env python

import rospy
from rospkg import RosPack
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, SetBool, TriggerResponse, SetBoolResponse
from hebi_cpp_api_examples.msg import FlipperVelocityCommand, TorqueModeCommand, TreadedBaseState
import os
from enum import Enum, auto

import numpy as np
from scipy.spatial.transform import Rotation as R

import hebi

import typing
if typing.TYPE_CHECKING:
    from typing import Optional
    import numpy.typing as npt
    from hebi._internal.group import Group

def printlog(level, msg, *args, **kwargs):
    node_name = rospy.get_name()
    formatted_msg = f"[{node_name}] {msg}"
    
    if level == rospy.DEBUG:
        rospy.logdebug(formatted_msg, *args, **kwargs)
    elif level == rospy.INFO:
        rospy.loginfo(formatted_msg, *args, **kwargs)
    elif level == rospy.WARN:
        rospy.logwarn(formatted_msg, *args, **kwargs)
    elif level == rospy.ERROR:
        rospy.logerr(formatted_msg, *args, **kwargs)
    elif level == rospy.FATAL:
        rospy.logfatal(formatted_msg, *args, **kwargs)


class TreadedBase:
    # FRAME CONVENTION:
    # ORIGIN = MID-POINT BETWEEN THE WHEELS
    # +X-AXIS = FORWARD
    # +Y-AXIS = LEFT
    # +Z-AXIS = UP

    #   Left  |   Right
    #   1     |    2
    #         |
    #         |
    #   3     |    4

    WHEEL_DIAMETER = 0.125 # m
    WHEEL_BASE = 0.285 # m

    WHEEL_RADIUS = WHEEL_DIAMETER / 2

    TORSO_TORQUE_SCALE = 2.5 # Nm
    TORQUE_MAX = 25 # Nm
    FLIPPER_HOME_POS = np.pi/3

    def __init__(self, group: 'Group', chassis_ramp_time: float, flipper_ramp_time: float):
        self.group = group

        self.fbk = self.group.get_next_feedback()
        while self.fbk == None:
            self.fbk = self.group.get_next_feedback()

        self.wheel_fbk = self.fbk.create_view([0, 1, 2, 3])
        self.flipper_fbk = self.fbk.create_view([4, 5, 6, 7])
        self.cmd = hebi.GroupCommand(group.size)
        self.wheel_cmd = self.cmd.create_view([0, 1, 2, 3])
        self.flipper_cmd = self.cmd.create_view([4, 5, 6, 7])

        self.flipper_sign = np.array([-1, 1, 1, -1])

        self.chassis_ramp_time = chassis_ramp_time
        self.flipper_ramp_time = flipper_ramp_time

        self.flipper_cmd.position = self.flipper_fbk.position
        self.wheel_cmd.position = np.nan

        self.t_prev: float = rospy.Time.now().to_sec()

        self.chassis_traj = None
        self.flipper_traj = None

        self.robot_model = None
    
    @property
    def mstop_pressed(self):
        return any(self.fbk.mstop_state == 0)

    @property
    def has_active_base_trajectory(self):
        if self.chassis_traj is not None and self.t_prev < self.chassis_traj.end_time:
            return True
        return False
    
    @property
    def has_active_flipper_trajectory(self):
        if self.flipper_traj is not None and self.t_prev < self.flipper_traj.end_time:
            return True
        return False

    @property
    def has_active_trajectory(self):
        return self.has_active_base_trajectory or self.has_active_flipper_trajectory
    
    @property
    def wheel_to_chassis_vel(self) -> 'npt.NDArray[np.float64]':
        wr = self.WHEEL_RADIUS / (self.WHEEL_BASE / 2)
        return np.array([
            [self.WHEEL_RADIUS, -self.WHEEL_RADIUS, self.WHEEL_RADIUS, -self.WHEEL_RADIUS],
            [0, 0, 0, 0],
            [wr, wr, wr, wr]
        ])

    @property
    def chassis_to_wheel_vel(self) -> 'npt.NDArray[np.float64]':
        return np.array([
            [1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / self.WHEEL_DIAMETER],
            [-1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / self.WHEEL_DIAMETER],
            [1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / self.WHEEL_DIAMETER],
            [-1 / self.WHEEL_RADIUS, 0, self.WHEEL_BASE / self.WHEEL_DIAMETER],
        ])

    @property
    def aligned_flipper_position(self) -> 'npt.NDArray[np.float64]':
        mean_pos = np.mean(np.abs(self.flipper_fbk.position))
        return np.array([-mean_pos, mean_pos, mean_pos, -mean_pos], dtype=np.float64)

    @property
    def flipper_height(self) -> 'npt.NDArray[np.float64]':
        x = np.cos(self.flipper_sign * np.pi / 4 + self.flipper_fbk.position)
        return (1 + self.WHEEL_BASE * np.clip(x, 0, 1) / self.WHEEL_RADIUS)
    
    @property
    def pose(self) -> 'npt.NDArray[np.float64]':
        # Use Pose estimate of a single flipper actuator in Tready to get the body Pose estimate
        pos = self.fbk.position
        position = []
        for idx in range(0, 4):
            position.append(pos[idx+4])
            position.append(pos[idx])

        frames = self.robot_model.get_forward_kinematics_mat('com', position)
        track_rot_mat = frames[10, :3, :3]

        quat = self.fbk.orientation[1]
        q_track = np.array([quat[1], quat[2], quat[3], quat[0]])
        rot_mat_tready = R.from_quat(q_track).as_matrix()
        rot_mat_tready = rot_mat_tready @ track_rot_mat.T

        # convert to euler
        rpy = R.from_matrix(rot_mat_tready).as_euler('xyz', degrees=True)
        return rpy
    
    def update_feedback(self):
        self.group.get_next_feedback(reuse_fbk=self.fbk)

    def update(self, t_now: float, get_feedback: bool = True):
        if get_feedback:
            self.group.get_next_feedback(reuse_fbk=self.fbk)

        if self.flipper_traj is None and self.chassis_traj is None:
            # printlog(rospy.WARN, "No trajectories, zeroing velocity")
            self.cmd.velocity = 0.0
        else:
            if self.chassis_traj is not None:
                # chassis update
                t = min(t_now, self.chassis_traj.end_time)
                [_, vel, _] = self.chassis_traj.get_state(t)

                flipper_height = self.flipper_height
                # Moving average setup below, in an attempt to make Tready less wobbly on tiptoes
                gyro_z = self.flipper_fbk.gyro[:, 2]

                self.wheel_cmd.position[:] = np.nan
                self.wheel_cmd.velocity = self.chassis_to_wheel_vel @ vel + gyro_z * flipper_height
                for i in range(len(self.wheel_cmd.effort)):
                    if flipper_height[i] > 1 - 1e-12:
                        self.wheel_cmd.effort[i] = np.nan
                    else:
                        self.wheel_cmd.effort[i] = self.flipper_sign[i] * np.tanh(-flipper_height[i] * (1 + self.WHEEL_BASE / self.WHEEL_RADIUS))

            if self.flipper_traj is not None:
                # flipper update
                t = min(t_now, self.flipper_traj.end_time)
                [_, vel, _] = self.flipper_traj.get_state(t)

                self.flipper_cmd.velocity = vel
                self.flipper_cmd.position += vel * (t_now - self.t_prev)

        self.t_prev = t_now

    def send(self):
        self.group.send_command(self.cmd)
    
    def set_flipper_cmd(self, p=None, v=None, e=None):
        if p is not None:
            self.flipper_cmd.position = p
        if v is not None:
            self.flipper_cmd.velocity = v
        if e is not None:
            self.flipper_cmd.effort = e
    
    def set_chassis_cmd(self, p=None, v=None, e=None):
        if p is not None:
            self.wheel_cmd.position = p
        if v is not None:
            self.wheel_cmd.velocity = v
        if e is not None:
            self.wheel_cmd.effort = e

    def set_flipper_trajectory(self, t_now: float, ramp_time: float, p=None, v=None):
        times = [t_now, t_now + ramp_time]
        positions = np.empty((4, 2), dtype=np.float64)
        velocities = np.empty((4, 2), dtype=np.float64)
        accelerations = np.empty((4, 2), dtype=np.float64)

        if self.flipper_traj is not None:
            t = min(t_now, self.flipper_traj.end_time)
            positions[:, 0], velocities[:, 0], accelerations[:, 0] = self.flipper_traj.get_state(t)
        else:
            positions[:, 0] = self.flipper_fbk.position
            velocities[:, 0] = self.flipper_fbk.velocity
            accelerations[:, 0] = self.flipper_fbk.effort_command

        positions[:, 1] = np.nan if p is None else p
        velocities[:, 1] = 0.0 if v is None else v
        accelerations[:, 1] = 0.0

        self.flipper_traj = hebi.trajectory.create_trajectory(times, positions, velocities, accelerations)

    def set_chassis_vel_trajectory(self, t_now: float, ramp_time: float, v):
        times = [t_now, t_now + ramp_time]
        positions = np.empty((3, 2))
        velocities = np.empty((3, 2))
        efforts = np.empty((3, 2))

        if self.chassis_traj is not None:
            t = min(t_now, self.chassis_traj.end_time)
            positions[:, 0], velocities[:, 0], efforts[:, 0] = self.chassis_traj.get_state(t)
        else:
            positions[:, 0] = 0.0
            velocities[:, 0] = self.wheel_to_chassis_vel @ self.wheel_fbk.velocity
            efforts[:, 0] = self.wheel_to_chassis_vel @ self.wheel_fbk.effort

        positions[:, 1] = np.nan
        velocities[:, 1] = v
        efforts[:, 1] = 0.0

        self.chassis_traj = hebi.trajectory.create_trajectory(times, positions, velocities, efforts)

    def home(self, t_now: float):
        flipper_home = self.flipper_sign * self.FLIPPER_HOME_POS
        self.set_chassis_vel_trajectory(t_now, 0.25, [0, 0, 0])
        self.set_flipper_trajectory(t_now, 3.0, p=flipper_home)

    def align_flippers(self, t_now: float):
        self.set_flipper_trajectory(t_now, 3.0, p=self.aligned_flipper_position)
    
    def set_robot_model(self, hrdf_file: str):
        self.robot_model = hebi.robot_model.import_from_hrdf(hrdf_file)

    def set_color(self, color: 'hebi.Color | str'):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = color
        self.group.send_command(color_cmd)

    def clear_color(self):
        color_cmd = hebi.GroupCommand(self.group.size)
        color_cmd.led.color = hebi.Color(0, 0, 0, 0)
        self.group.send_command(color_cmd)

class TreadyControlState(Enum):
    STARTUP = auto()
    HOMING = auto()
    ALIGNING = auto()
    TELEOP = auto()
    EMERGENCY_STOP = auto()
    EXIT = auto()

class TreadyInputs:
    def __init__(self, home: bool = False, base_motion: Twist = Twist(), flippers: 'list[float]' = [0, 0, 0, 0], align_flippers: bool = False, stable_mode: bool = False, torque_toggle: bool = False):
        self.home = home
        self.base_motion = base_motion
        self.flippers = flippers
        self.align_flippers = align_flippers
        self.stable_mode = stable_mode
        self.torque_toggle = torque_toggle
    
    def __repr__(self) -> str:
        return f'TreadyInputs(home={self.home}, base_motion={self.base_motion}, flippers={self.flippers}, align_flippers={self.align_flippers}, stable_mode={self.stable_mode}, torque_toggle={self.torque_toggle})'

class TreadyControl:

    FLIPPER_VEL_SCALE = 1  # rad/sec

    def __init__(self, base: TreadedBase):
        self.namespace = ''
        
        self.state = TreadyControlState.STARTUP
        self.base = base

        self.SPEED_MAX_LIN = 0.45  # m/s
        self.SPEED_MAX_ROT = self.SPEED_MAX_LIN / (base.WHEEL_BASE / 2) # rad/s

        self.set_default_torque_params()

    def set_default_torque_params(self):
        self.torque_max = self.base.TORQUE_MAX / 2
        self.torque_angle = np.pi/8
        self.roll_adjust = 1
        self.pitch_adjust = 1

    @property
    def running(self):
        return self.state is not self.state.EXIT

    def start_logging(self):
        self.base.group.start_log("logs", mkdirs=True)

    def cycle_log(self):
        self.base.group.stop_log()
        self.base.group.start_log("logs", mkdirs=True)

    def send(self):
        self.base.send()
    
    def set_torque_max(self, torque: float):
        if not np.isfinite(torque):
            printlog(rospy.ERROR, self.namespace + "Torque must be finite")
            return
        self.torque_max = min(self.base.TORQUE_MAX, torque)
    
    def set_torque_angle(self, angle: float):
        if not np.isfinite(angle):
            printlog(rospy.ERROR, self.namespace + "Angle must be finite")
            return
        self.torque_angle = np.clip(angle, 0, np.pi/2)
    
    def set_roll_adjust(self, adjust: float):
        if not np.isfinite(adjust):
            printlog(rospy.ERROR, self.namespace + "Roll adjustment must be finite")
            return
        self.roll_adjust = np.clip(adjust, 0, 1)
    
    def set_pitch_adjust(self, adjust: float):
        if not np.isfinite(adjust):
            printlog(rospy.ERROR, self.namespace + "Pitch adjustment must be finite")
            return
        self.pitch_adjust = np.clip(adjust, 0, 1)

    def update(self, t_now: float, tready_input: 'Optional[TreadyInputs]'=None):
        self.base.update_feedback()

        if self.state is self.state.EXIT:
            return
        
        if self.base.mstop_pressed and self.state is not self.state.EMERGENCY_STOP:
            self.transition_to(t_now, self.state.EMERGENCY_STOP)
            return

        if self.state is self.state.EMERGENCY_STOP:
            if not self.base.mstop_pressed:
                printlog(rospy.INFO, self.namespace + "Emergency Stop Released")
                self.transition_to(t_now, self.state.TELEOP)
        
        # After startup, transition to homing
        elif self.state is self.state.STARTUP:
            self.transition_to(t_now, self.state.HOMING)

        # If homing/aligning is complete, transition to teleop
        elif self.state is self.state.HOMING or self.state is self.state.ALIGNING:
            if not self.base.has_active_flipper_trajectory:
                self.transition_to(t_now, self.state.TELEOP)

        # Teleop mode
        elif self.state is self.state.TELEOP:
            if tready_input is None:
                self.base.flipper_traj = None
                self.base.chassis_traj = None
            # Check for home button
            elif tready_input.home:
                if tready_input.stable_mode:
                    printlog(rospy.ERROR, self.namespace + "Cannot home in torque mode")
                    return
                self.transition_to(t_now, self.state.HOMING)
            # Check for flipper alignment
            elif tready_input.align_flippers:
                if tready_input.stable_mode:
                    printlog(rospy.ERROR, self.namespace + "Cannot align flippers in torque mode")
                    return
                self.transition_to(t_now, self.state.ALIGNING)
            else:
                if tready_input.stable_mode:
                    roll_angle, pitch_angle, _ = self.base.pose

                    if roll_angle > 0:
                        roll_torque = np.array([0, -1, 0, 1]) * roll_angle * self.base.TORSO_TORQUE_SCALE * self.roll_adjust
                    else:
                        roll_torque = np.array([-1, 0, 1, 0]) * roll_angle * self.base.TORSO_TORQUE_SCALE * self.roll_adjust
                    
                    if pitch_angle > 0:
                        pitch_torque = np.array([1, -1, 0, 0]) * pitch_angle * self.base.TORSO_TORQUE_SCALE * self.pitch_adjust
                    else:
                        pitch_torque = np.array([0, 0, 1, -1]) * pitch_angle * self.base.TORSO_TORQUE_SCALE * self.pitch_adjust
                    
                    level_torque = roll_torque + pitch_torque
                    flipper_efforts = -np.tanh(self.base.flipper_fbk.position - self.base.flipper_sign * self.torque_angle) * self.torque_max + level_torque

                    self.base.flipper_traj = None
                    self.base.set_flipper_cmd(p=np.ones(4) * np.nan, v = np.ones(4) * np.nan, e=flipper_efforts)
                else:
                    # Flipper Control
                    flipper_vels = self.base.flipper_sign * tready_input.flippers * self.FLIPPER_VEL_SCALE

                    if tready_input.torque_toggle:
                        self.base.set_flipper_cmd(p=self.base.flipper_fbk.position, e=np.ones(4) * np.nan)
                    self.base.set_flipper_trajectory(t_now, self.base.flipper_ramp_time, v=flipper_vels)
                
                # Mobile Base Control
                chassis_vels = np.array([
                    np.sign(tready_input.base_motion.linear.x) * min(self.SPEED_MAX_LIN, abs(tready_input.base_motion.linear.x)),
                    0,
                    - np.sign(tready_input.base_motion.angular.z) * min(self.SPEED_MAX_ROT, abs(tready_input.base_motion.angular.z))
                ], dtype=np.float64)

                self.base.set_chassis_vel_trajectory(t_now, self.base.chassis_ramp_time, chassis_vels)
        
        self.base.update(t_now, get_feedback=False)

    def transition_to(self, t_now: float, state: TreadyControlState):
        # self transitions are noop
        if state == self.state:
            return

        if state is self.state.HOMING:
            printlog(rospy.INFO, self.namespace + "TRANSITIONING TO HOMING")
            self.base.set_color('magenta')
            self.base.home(t_now)
        
        elif state is self.state.ALIGNING:
            printlog(rospy.INFO, self.namespace + "TRANSITIONING TO ALIGNING")
            self.base.set_color('magenta')
            self.base.align_flippers(t_now)

        elif state is self.state.TELEOP:
            printlog(rospy.INFO, self.namespace + "TRANSITIONING TO TELEOP")
            self.base.set_color('transparent')
        
        elif state is self.state.EMERGENCY_STOP:
            printlog(rospy.WARN, self.namespace + "Emergency Stop Pressed, disabling motion")
            self.base.set_color('yellow')
            self.base.chassis_traj = None
            self.base.flipper_traj = None

        elif state is self.state.EXIT:
            printlog(rospy.INFO, self.namespace + "TRANSITIONING TO EXIT")
            self.base.set_color('red')

        self.state = state
    
    def stop(self):
        self.transition_to(rospy.Time.now().to_sec(), self.state.EXIT)


def load_gains(group, gains_file):
    gains_command = hebi.GroupCommand(group.size)
    rospy.sleep(0.1)
    try:
        gains_command.read_gains(gains_file)
    except Exception as e:
        printlog(rospy.WARN, f'Warning - Could not load gains: {e}')
        return False

    # Send gains multiple times
    for _ in range(3):
        group.send_command(gains_command)
        rospy.sleep(0.1)
    
    return True


class TreadedBaseNode:
    def __init__(self):
        # Load parameters
        gains_package = rospy.get_param('~gains_package', None)
        gains_file = rospy.get_param('~gains_file', None)
        hrdf_package = rospy.get_param('~hrdf_package', None)
        hrdf_file = rospy.get_param('~hrdf_file', None)

        if not gains_package or not gains_file:
            rospy.logerr("Could not find/read required 'gains_package' or 'gains_file' parameter; aborting!")
            return None
    
        if not hrdf_package or not hrdf_file:
            rospy.logerr("Could not find/read required 'hrdf_package' or 'hrdf_file' parameter; aborting!")
            return None

        lookup = hebi.Lookup()
        rospy.sleep(2)

        family = "Tready"
        flipper_names = [f'T{n+1}_J1_flipper' for n in range(4)]
        wheel_names = [f'T{n+1}_J2_track' for n in range(4)]

        # Create self.base group
        base_group = lookup.get_group_from_names(family, wheel_names + flipper_names)
        while base_group is None and not rospy.is_shutdown():
            printlog(rospy.WARN, 'Looking for Tready modules...')
            rospy.sleep(1)
            base_group = lookup.get_group_from_names(family, wheel_names + flipper_names)
        
        gains_path = os.path.join(RosPack().get_path(gains_package), gains_file)
        if not load_gains(base_group, gains_path):
            rospy.logerr("Failed to load gains!")

        self.base = TreadedBase(base_group, chassis_ramp_time=0.33, flipper_ramp_time=0.1)
        hrdf_path = os.path.join(RosPack().get_path(hrdf_package), hrdf_file)
        self.base.set_robot_model(hrdf_path)
        self.base_control = TreadyControl(self.base)

        # Initialize ROS interface
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("flipper_vel", FlipperVelocityCommand, self.flipper_vel_callback)
        rospy.Subscriber("torque_cmd", TorqueModeCommand, self.torque_cmd_callback)
        rospy.Subscriber("color", ColorRGBA, self.color_callback)

        rospy.Service("home_flippers", Trigger, self.home_service)
        rospy.Service("align_flippers", Trigger, self.align_service)
        rospy.Service("stable_mode", SetBool, self.torque_service)

        self.state_publisher = rospy.Publisher("state", TreadedBaseState, queue_size=100)

        self.input = None
        self.stable_mode = False

        self.home = False
        self.align_flippers = False

        self.last_cmd_time = 0.

    def home_service(self, req):
        if self.base_control.state is TreadyControlState.ALIGNING:
            return TriggerResponse(success=False, message="Cannot home while aligning!")
        if self.base_control.state is TreadyControlState.HOMING:
            return TriggerResponse(success=False, message="Already homing!")
        if self.stable_mode:
            return TriggerResponse(success=False, message="Cannot home in torque mode!")
        
        self.home = True
        self.last_cmd_time = rospy.Time.now().to_sec()

        return TriggerResponse(success=True, message="Homing command sent!")

    def align_service(self, req):
        if self.base_control.state is TreadyControlState.HOMING:
            return TriggerResponse(success=False, message="Cannot align while homing!")
        if self.base_control.state is TreadyControlState.ALIGNING:
            return TriggerResponse(success=False, message="Already aligning!")
        if self.stable_mode:
            return TriggerResponse(success=False, message="Cannot align in torque mode!")
        
        self.align_flippers = True
        self.last_cmd_time = rospy.Time.now().to_sec()
        
        return TriggerResponse(success=True, message="Alignment command sent!")
    
    def torque_service(self, req):
        if self.base_control.state is TreadyControlState.HOMING:
            return SetBoolResponse(success=False, message="Cannot toggle torque mode while homing!")
        if self.base_control.state is TreadyControlState.ALIGNING:
            return SetBoolResponse(success=False, message="Cannot toggle torque mode while aligning!")
        
        if req.data and self.stable_mode:
            return SetBoolResponse(success=False, message="Already in torque mode!")
        if not req.data and not self.stable_mode:
            return SetBoolResponse(success=False, message="Already in velocity mode!")
        
        self.stable_mode = req.data
        if self.input is None:
            self.input = TreadyInputs()        
        self.input.stable_mode = self.stable_mode
        self.input.torque_toggle = True
        self.last_cmd_time = rospy.Time.now().to_sec()

        return SetBoolResponse(success=True, message="Torque mode toggled!")

    def cmd_vel_callback(self, cmd):
        if self.input is None:
            self.input = TreadyInputs()
        self.input.base_motion = cmd
        self.input.stable_mode = self.stable_mode
        self.last_cmd_time = rospy.Time.now().to_sec()

    def flipper_vel_callback(self, cmd):
        if self.stable_mode:
            return
        if self.input is None:
            self.input = TreadyInputs()
        self.input.flippers = [cmd.front_left, cmd.front_right, cmd.back_left, cmd.back_right]
        self.input.stable_mode = self.stable_mode
        self.last_cmd_time = rospy.Time.now().to_sec()

    def torque_cmd_callback(self, cmd):
        if not self.stable_mode:
            return
        
        if self.input is None:
            self.input = TreadyInputs()
        self.base_control.set_torque_max(cmd.torque_max)
        self.base_control.set_torque_angle(cmd.torque_angle)
        self.base_control.set_roll_adjust(cmd.roll_adjust)
        self.base_control.set_pitch_adjust(cmd.pitch_adjust)
        self.input.stable_mode = self.stable_mode

    def color_callback(self, color_cmd):
        self.base.set_color(hebi.Color(color_cmd.r, color_cmd.g, color_cmd.b, color_cmd.a))
    
    def publish_state(self):
        state_msg = TreadedBaseState()
        state_msg.state = self.base_control.state.value
        state_msg.base_trajectory_active = self.base.has_active_base_trajectory
        state_msg.flipper_trajectory_active = self.base.has_active_flipper_trajectory
        state_msg.mstop_pressed = self.base.mstop_pressed
        state_msg.stable_mode = self.stable_mode
        self.state_publisher.publish(state_msg)
    
    def update(self, t):
        if self.home:
            self.home = False
            self.input = TreadyInputs(home=True)
        if self.align_flippers:
            self.align_flippers = False
            self.input = TreadyInputs(align_flippers=True)
        self.base_control.update(t, self.input)
        self.base_control.send()
        self.publish_state()

        if self.input is not None and t - self.last_cmd_time > 0.25:
            if self.stable_mode:
                self.input = TreadyInputs(stable_mode=self.stable_mode)
            else:
                self.input = None


def main():
    rospy.init_node('treaded_base_node', anonymous=True)
    # Main loop
    rate = rospy.Rate(100)
    tready_node = TreadedBaseNode()
    if tready_node is None:
        rospy.signal_shutdown('Failed to initialize Treaded Base Node')
    else:
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec()
            tready_node.update(t)
            rate.sleep()
        
        tready_node.base_control.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
