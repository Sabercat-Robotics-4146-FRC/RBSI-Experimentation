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

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;

  private final boolean isTurnMotorInverted;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        // Front Left
        driveSparkMax = new CANSparkMax(kFrontLeftDriveMotorId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kFrontLeftSteerMotorId, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(kFrontLeftEncoderId);
        absoluteEncoderOffset = new Rotation2d(kFrontLeftEncoderOffset);
        isTurnMotorInverted = kFrontLeftSteerInvert;
        break;

      case 1:
        // Front Right
        driveSparkMax = new CANSparkMax(kFrontRightDriveMotorId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kFrontRightSteerMotorId, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(kFrontRightEncoderId);
        absoluteEncoderOffset = new Rotation2d(kFrontRightEncoderOffset);
        isTurnMotorInverted = kFrontRightSteerInvert;
        break;

      case 2:
        // Back Left
        driveSparkMax = new CANSparkMax(kBackLeftDriveMotorId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kBackLeftSteerMotorId, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(kBackLeftEncoderId);
        absoluteEncoderOffset = new Rotation2d(kBackLeftEncoderOffset);
        isTurnMotorInverted = kBackLeftSteerInvert;
        break;

      case 3:
        // Back Right
        driveSparkMax = new CANSparkMax(kBackRightDriveMotorId, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(kBackRightSteerMotorId, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(kBackRightEncoderId);
        absoluteEncoderOffset = new Rotation2d(kBackRightEncoderOffset);
        isTurnMotorInverted = kBackRightSteerInvert;
        break;

      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit((int) kDriveCurrentLimit);
    turnSparkMax.setSmartCurrentLimit((int) kSteerCurrentLimit);
    driveSparkMax.enableVoltageCompensation(kOptimalVoltage);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / kDriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / kDriveGearRatio;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / kSteerGearRatio);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / kSteerGearRatio;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
