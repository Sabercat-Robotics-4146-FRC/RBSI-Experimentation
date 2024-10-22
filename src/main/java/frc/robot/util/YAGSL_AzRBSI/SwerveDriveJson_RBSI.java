// Copyright (c) 2024 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024 FRC 3481
// https://github.com/BroncBotz3481/YAGSL
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

package frc.robot.util.YAGSL_AzRBSI;

/**
 * {@link swervelib.SwerveDrive} JSON parsed class. Used to access parsed data from the
 * swervedrive.json file.
 */
public class SwerveDriveJson_RBSI {

  /** Robot IMU used to determine heading of the robot. */
  public DeviceJson_RBSI imu;

  /** Invert the IMU of the robot. */
  public boolean invertedIMU;

  /** Module JSONs in order clockwise order starting from front left. */
  public String[] modules;
}
