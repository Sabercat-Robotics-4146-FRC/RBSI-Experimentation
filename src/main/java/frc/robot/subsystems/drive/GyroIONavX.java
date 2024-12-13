// Copyright (c) 2024 Az-FIRST
// http://github.com/AZ-First
// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
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

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;

/** IO implementation for Pigeon2 */
public class GyroIONavX implements GyroIO<AHRS> {
  private final AHRS navx;

  // Constructor, taking default values
  public GyroIONavX() {
    navx = new AHRS(SPI.Port.kMXP, (byte) 100.0);
    navx.reset();
    // Set Angle Adjustment based on alliance
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      navx.setAngleAdjustment(0.0);
    } else {
      navx.setAngleAdjustment(180.0);
    }
  }

  // Return the Pigeon2 instance
  @Override
  public AHRS getGyro() {
    return navx;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navx.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(navx.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navx.getRate());
  }

  /**
   * Zero the NavX
   *
   * <p>This method should always rezero the pigeon in ALWAYS-BLUE-ORIGIN orientation. Testing,
   * however, shows that it's not doing what I think it should be doing. There is likely
   * interference with something else in the odometry
   */
  @Override
  public void zero() {
    // With the Pigeon facing forward, forward depends on the alliance selected.
    // Set Angle Adjustment based on alliance
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      navx.setAngleAdjustment(0.0);
    } else {
      navx.setAngleAdjustment(180.0);
    }
    System.out.println("Setting YAW to " + navx.getAngleAdjustment());
    navx.zeroYaw();
  }
}
