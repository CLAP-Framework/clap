import rospy
import numpy as np

from zzz_control_msgs.msg import ControlCommand, AuxiliaryCommand
from dbw_mkz_msgs.msg import ThrottleCmd, BrakeCmd, SteeringCmd, GearCmd, TurnSignalCmd,\
    Gear, TurnSignal

def to_mkz_throttle(cmd, map_command, map_actuator):
    assert type(cmd) == ControlCommand

    if cmd.accel >= 0:
        new_cmd = ThrottleCmd()
        new_cmd.enable = True
        new_cmd.pedal_cmd = np.interp(cmd.accel, map_command, map_actuator)
        new_cmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        return new_cmd
    else:
        return None

def to_mkz_brake(cmd, map_command, map_actuator):
    assert type(cmd) == ControlCommand

    if cmd.accel < 0:
        new_cmd = BrakeCmd()
        new_cmd.enable = True
        new_cmd.pedal_cmd = np.interp(-cmd.accel, map_command, map_actuator)
        new_cmd.pedal_cmd_type = BrakeCmd.CMD_DECEL
        return new_cmd
    else:
        return None

def to_mkz_steering(cmd, map_command, map_actuator):
    assert type(cmd) == ControlCommand

    new_cmd = SteeringCmd()
    new_cmd.enable = True
    new_cmd.steering_wheel_angle_cmd = np.interp(cmd.steer, map_command, map_actuator)
    new_cmd.cmd_type = SteeringCmd.CMD_ANGLE
    return new_cmd

def to_mkz_gear(cmd):
    assert type(cmd) == ControlCommand

    new_cmd = GearCmd()
    new_cmd.enable = True
    if cmd.gear > 0:
        new_cmd.cmd.gear = Gear.DRIVE
    elif cmd.gear == ControlCommand.GEAR_NONE:
        new_cmd.cmd.gear = Gear.NONE
    elif cmd.gear == ControlCommand.GEAR_PARKING:
        new_cmd.cmd.gear = Gear.PARK
    elif cmd.gear == ControlCommand.GEAR_REVERSE:
        new_cmd.cmd.gear = Gear.REVERSE
    elif cmd.gear == ControlCommand.GEAR_NEUTRAL:
        new_cmd.cmd.gear = Gear.NEUTRAL
    elif cmd.gear == ControlCommand.GEAR_DRIVE:
        new_cmd.cmd.gear = Gear.DRIVE
    else:
        rospy.logerr("Incorrect gear command value!")
        return None

    return new_cmd

def to_mkz_turn_signal(cmd, pubs):
    assert type(cmd) == AuxiliaryCommand

    new_cmd = TurnSignalCmd()
    if cmd.blinker_flag == AuxiliaryCommand.BLINKER_OFF:
        new_cmd.cmd = TurnSignal.NONE
    elif cmd.blinker_flag == AuxiliaryCommand.BLINKER_LEFT:
        new_cmd.cmd = TurnSignal.LEFT
    elif cmd.blinker_flag == AuxiliaryCommand.BLINKER_RIGHT:
        new_cmd.cmd = TurnSignal.RIGHT
    elif cmd.blinker_flag == AuxiliaryCommand.BLINKER_LEFT | AuxiliaryCommand.BLINKER_RIGHT:
        new_cmd.cmd = TurnSignal.LEFT | TurnSignal.RIGHT
    else:
        rospy.logerr("Incorrect turning signal command value!")
        return None

    return new_cmd
