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

package frc.robot.subsystems.accelerometer;

import static frc.robot.Constants.AccelerometerConstants.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

/** Accelerometer subsystem (built upon a virtual subsystem) */
public class Accelerometer extends VirtualSubsystem {

  private final BuiltInAccelerometer rioAccelerometer = new BuiltInAccelerometer();
  private final Pigeon2 pigeonAccelerometer;

  // Define the 3D vectors needed to hold values
  private Translation3d rioAccVector;
  private Translation3d pigeonAccVector;
  private Translation3d prevRioAccel = new Translation3d();
  private Translation3d prevPigeonAccel = new Translation3d();
  private Translation3d rioJerkVector;
  private Translation3d pigeonJerkVector;

  // Class method definition
  public Accelerometer(Pigeon2 pigeonAccelerometer) {
    this.pigeonAccelerometer = pigeonAccelerometer;
  }

  /** Get accelerations, compute jerks, log everything */
  public void periodic() {

    long start = System.nanoTime();

    // Compute the Rio's acceleration, rotated as needed in the XY plane (yaw)
    // RoboRio provides accelerations in `g`, from -8 to +8; convert to m/s^2
    rioAccVector =
        new Translation3d(rioAccelerometer.getX(), rioAccelerometer.getY(), rioAccelerometer.getZ())
            .rotateBy(new Rotation3d(0., 0., kRioOrientation.getRadians()))
            .times(9.81);

    // Compute the Pigeon's acceleration, rotated as needed
    // Pigeon provides accelerations in `g`, from -2 to +2; convert to m/s^2
    pigeonAccVector =
        new Translation3d(
                pigeonAccelerometer.getAccelerationX().getValueAsDouble(),
                pigeonAccelerometer.getAccelerationY().getValueAsDouble(),
                pigeonAccelerometer.getAccelerationZ().getValueAsDouble())
            .rotateBy(new Rotation3d(0., 0., kPigeonOrientation.getRadians()))
            .times(9.81);

    // Compute the jerks ((current - prev accel) / loop time)
    rioJerkVector = rioAccVector.minus(prevRioAccel).div(Constants.loopPeriodSecs);
    pigeonJerkVector = pigeonAccVector.minus(prevPigeonAccel).div(Constants.loopPeriodSecs);

    // Log everything to both AdvantageKit and SmartDashboard
    Logger.recordOutput("Acceleration/Rio/Accel_mss", rioAccVector);
    Logger.recordOutput("Acceleration/Rio/Jerk_msss", rioJerkVector);
    Logger.recordOutput("Acceleration/Pigeon/Accel_mss", pigeonAccVector);
    Logger.recordOutput("Acceleration/Pigeon/Jerk_msss", pigeonJerkVector);
    SmartDashboard.putNumber("RioXAccel", rioAccVector.getX());
    SmartDashboard.putNumber("RioYAccel", rioAccVector.getY());
    SmartDashboard.putNumber("PigeonXAccel", pigeonAccVector.getX());
    SmartDashboard.putNumber("PigeonYAccel", pigeonAccVector.getY());

    // Set the "previous" accelerations to the current for the next loop
    prevRioAccel = rioAccVector;
    prevPigeonAccel = pigeonAccVector;

    long finish = System.nanoTime();
    long timeElapsed = finish - start;
    Logger.recordOutput("LoggedRobot/AccelCodeMS", (double) timeElapsed / 1.e6);
  }
}
