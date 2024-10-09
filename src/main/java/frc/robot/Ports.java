// Copyright (c) 2024 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024 FRC 1678
// https://github.com/frc1678
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import frc.robot.util.CanDeviceId;

/** List of Channel and CAN IDs */
public class Ports {

  /* DRIVETRAIN CAN DEVICE IDS */
  // This is the default setup for the Az-RBSI swerve base
  // Swerve Modules go:
  // FL,FR,BL,BR
  //
  // 0 1
  // 2 3
  public static final CanDeviceId FL_DRIVE = new CanDeviceId(1, "canivore");
  public static final CanDeviceId FL_ROTATION = new CanDeviceId(2, "canivore");
  public static final CanDeviceId FL_CANCODER = new CanDeviceId(3, "canivore");

  public static final CanDeviceId FR_DRIVE = new CanDeviceId(4, "canivore");
  public static final CanDeviceId FR_ROTATION = new CanDeviceId(5, "canivore");
  public static final CanDeviceId FR_CANCODER = new CanDeviceId(6, "canivore");

  public static final CanDeviceId BL_DRIVE = new CanDeviceId(7, "canivore");
  public static final CanDeviceId BL_ROTATION = new CanDeviceId(8, "canivore");
  public static final CanDeviceId BL_CANCODER = new CanDeviceId(9, "canivore");

  public static final CanDeviceId BR_DRIVE = new CanDeviceId(10, "canivore");
  public static final CanDeviceId BR_ROTATION = new CanDeviceId(11, "canivore");
  public static final CanDeviceId BR_CANCODER = new CanDeviceId(12, "canivore");

  public static final CanDeviceId PIGEON = new CanDeviceId(13, "canivore");

  /* POWER DISTRIBUTION CAN ID */
  public static final CanDeviceId POWER_CAN_DEVICE_ID = new CanDeviceId(1, "");

  /* SUBSYSTEM CAN DEVICE IDS */
  // This is where mechanism subsystem devices are defined
  // Example:
  // public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(3, "");

  /* BEAM BREAK and/or LIMIT SWITCH DIO CHANNELS */
  // This is where digital I/O feedback devices are defined
  // Example:
  // public static final int ELEVATOR_BOTTOM_LIMIT = 3;

  /* LINEAR SERVO PWM CHANNELS */
  // This is where PWM-controlled devices (actuators, servos, pneumatics, etc.)
  // are defined
  // Example:
  // public static final int INTAKE_SERVO = 0;
}
