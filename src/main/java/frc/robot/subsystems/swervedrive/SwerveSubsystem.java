// Copyright (c) 2024 Az-FIRST
// http://github.com/AZ-First
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
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.SwerveType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.swervedrive.underlying.Phoenix6Swerve;
import frc.robot.subsystems.swervedrive.underlying.YAGSLSwerve;
import java.io.File;

/**
 * Az-RBSI swerve class that generalizes the swerve subsystem to allow for the use of either
 * Phoenix6 or YAGSL underlying libraries
 */
public class SwerveSubsystem extends Phoenix6Swerve {

  private YAGSLSwerve y_swerve;

  public SwerveSubsystem(Enum<SwerveType> swerveType) {

    if (swerveType == SwerveType.YAGSL) {

      // Load up the YASGL things into this class
      y_swerve = new YAGSLSwerve(new File(Filesystem.getDeployDirectory(), "swerve"));

    } else if (swerveType == SwerveType.PHOENIX6) {

      // Load up the Phoenix6 things into this class
      p_swerve = TunerConstants.DriveTrain;

    } else {

      // Raise an error because something unknown was passed

    }
  }

  //
}
