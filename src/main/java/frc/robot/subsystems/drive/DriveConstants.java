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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.YagslConstants;

/**
 * Holds the proper set of drive constants given the type of drive
 *
 * <p>Does some translation of how the two keep values to return a completely unified API. This file
 * should not be modified.
 */
public class DriveConstants {

  // Declare all the constants
  public static final double kCoupleRatio;
  public static final double kDriveGearRatio;
  public static final double kSteerGearRatio;
  public static final double kWheelRadiusInches;
  public static final String kCANbusName;
  public static final int kPigeonId;
  public static final double kSteerInertia;
  public static final double kDriveInertia;
  public static final double kSteerFrictionVoltage;
  public static final double kDriveFrictionVoltage;
  public static final double kSteerCurrentLimit;
  public static final double kDriveCurrentLimit;
  public static final double kOptimalVoltage;
  public static final int kFrontLeftDriveMotorId;
  public static final int kFrontLeftSteerMotorId;
  public static final int kFrontLeftEncoderId;
  public static final String kFrontLeftDriveCanbus;
  public static final String kFrontLeftSteerCanbus;
  public static final String kFrontLeftEncoderCanbus;
  public static final String kFrontLeftDriveType;
  public static final String kFrontLeftSteerType;
  public static final String kFrontLeftEncoderType;
  public static final double kFrontLeftEncoderOffset; // In Radians
  public static final boolean kFrontLeftDriveInvert;
  public static final boolean kFrontLeftSteerInvert;
  public static final boolean kFrontLeftEncoderInvert;
  public static final double kFrontLeftXPosInches;
  public static final double kFrontLeftYPosInches;
  public static final int kFrontRightDriveMotorId;
  public static final int kFrontRightSteerMotorId;
  public static final int kFrontRightEncoderId;
  public static final String kFrontRightDriveCanbus;
  public static final String kFrontRightSteerCanbus;
  public static final String kFrontRightEncoderCanbus;
  public static final String kFrontRightDriveType;
  public static final String kFrontRightSteerType;
  public static final String kFrontRightEncoderType;
  public static final double kFrontRightEncoderOffset; // In Radians
  public static final boolean kFrontRightDriveInvert;
  public static final boolean kFrontRightSteerInvert;
  public static final boolean kFrontRightEncoderInvert;
  public static final double kFrontRightXPosInches;
  public static final double kFrontRightYPosInches;
  public static final int kBackLeftDriveMotorId;
  public static final int kBackLeftSteerMotorId;
  public static final int kBackLeftEncoderId;
  public static final String kBackLeftDriveCanbus;
  public static final String kBackLeftSteerCanbus;
  public static final String kBackLeftEncoderCanbus;
  public static final String kBackLeftDriveType;
  public static final String kBackLeftSteerType;
  public static final String kBackLeftEncoderType;
  public static final double kBackLeftEncoderOffset; // In Radians
  public static final boolean kBackLeftDriveInvert;
  public static final boolean kBackLeftSteerInvert;
  public static final boolean kBackLeftEncoderInvert;
  public static final double kBackLeftXPosInches;
  public static final double kBackLeftYPosInches;
  public static final int kBackRightDriveMotorId;
  public static final int kBackRightSteerMotorId;
  public static final int kBackRightEncoderId;
  public static final String kBackRightDriveCanbus;
  public static final String kBackRightSteerCanbus;
  public static final String kBackRightEncoderCanbus;
  public static final String kBackRightDriveType;
  public static final String kBackRightSteerType;
  public static final String kBackRightEncoderType;
  public static final double kBackRightEncoderOffset; // In Radians
  public static final boolean kBackRightDriveInvert;
  public static final boolean kBackRightSteerInvert;
  public static final boolean kBackRightEncoderInvert;
  public static final double kBackRightXPosInches;
  public static final double kBackRightYPosInches;
  public static final double kDriveP;
  public static final double kDriveI;
  public static final double kDriveD;
  public static final double kDriveF;
  public static final double kDriveIZ;
  public static final double kSteerP;
  public static final double kSteerI;
  public static final double kSteerD;
  public static final double kSteerF;
  public static final double kSteerIZ;

  // Fill in the values from the proper source
  static {
    switch (Constants.getSwerveType()) {
      case PHOENIX6:
        kCoupleRatio = TunerConstants.FrontLeft.CouplingGearRatio;
        kDriveGearRatio = TunerConstants.FrontLeft.DriveMotorGearRatio;
        kSteerGearRatio = TunerConstants.FrontLeft.SteerMotorGearRatio;
        kWheelRadiusInches = TunerConstants.FrontLeft.WheelRadius;
        kCANbusName = TunerConstants.DrivetrainConstants.CANBusName;
        kPigeonId = TunerConstants.DrivetrainConstants.Pigeon2Id;
        kSteerInertia = TunerConstants.FrontLeft.SteerInertia;
        kDriveInertia = TunerConstants.FrontLeft.DriveInertia;
        kSteerFrictionVoltage = 0.0;
        kDriveFrictionVoltage = 0.1;
        kSteerCurrentLimit = 40.0; // Example from CTRE documentation
        kDriveCurrentLimit = 120.0; // Example from CTRE documentation
        kOptimalVoltage = 12.0; // Assumed Ideal
        kFrontLeftDriveMotorId = TunerConstants.FrontLeft.DriveMotorId;
        kFrontLeftSteerMotorId = TunerConstants.FrontLeft.SteerMotorId;
        kFrontLeftEncoderId = TunerConstants.FrontLeft.CANcoderId;
        kFrontLeftDriveCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kFrontLeftSteerCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kFrontLeftEncoderCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kFrontLeftDriveType = "kraken";
        kFrontLeftSteerType = "kraken";
        kFrontLeftEncoderType = "cancoder";
        kFrontLeftEncoderOffset =
            -Units.rotationsToRadians(TunerConstants.FrontLeft.CANcoderOffset) + Math.PI;
        kFrontLeftDriveInvert = TunerConstants.FrontLeft.DriveMotorInverted;
        kFrontLeftSteerInvert = TunerConstants.FrontLeft.SteerMotorInverted;
        kFrontLeftEncoderInvert = false;
        kFrontLeftXPosInches = TunerConstants.FrontLeft.LocationX;
        kFrontLeftYPosInches = TunerConstants.FrontLeft.LocationY;
        kFrontRightDriveMotorId = TunerConstants.FrontRight.DriveMotorId;
        kFrontRightSteerMotorId = TunerConstants.FrontRight.SteerMotorId;
        kFrontRightEncoderId = TunerConstants.FrontRight.CANcoderId;
        kFrontRightDriveCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kFrontRightSteerCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kFrontRightEncoderCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kFrontRightDriveType = "kraken";
        kFrontRightSteerType = "kraken";
        kFrontRightEncoderType = "cancoder";
        kFrontRightEncoderOffset =
            -Units.rotationsToRadians(TunerConstants.FrontRight.CANcoderOffset);
        kFrontRightDriveInvert = TunerConstants.FrontRight.DriveMotorInverted;
        kFrontRightSteerInvert = TunerConstants.FrontRight.SteerMotorInverted;
        kFrontRightEncoderInvert = false;
        kFrontRightXPosInches = TunerConstants.FrontRight.LocationX;
        kFrontRightYPosInches = TunerConstants.FrontRight.LocationY;
        kBackLeftDriveMotorId = TunerConstants.BackLeft.DriveMotorId;
        kBackLeftSteerMotorId = TunerConstants.BackLeft.SteerMotorId;
        kBackLeftEncoderId = TunerConstants.BackLeft.CANcoderId;
        kBackLeftDriveCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kBackLeftSteerCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kBackLeftEncoderCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kBackLeftDriveType = "kraken";
        kBackLeftSteerType = "kraken";
        kBackLeftEncoderType = "cancoder";
        kBackLeftEncoderOffset =
            -Units.rotationsToRadians(TunerConstants.BackLeft.CANcoderOffset) + Math.PI;
        kBackLeftDriveInvert = TunerConstants.BackLeft.DriveMotorInverted;
        kBackLeftSteerInvert = TunerConstants.BackLeft.SteerMotorInverted;
        kBackLeftEncoderInvert = false;
        kBackLeftXPosInches = TunerConstants.BackLeft.LocationX;
        kBackLeftYPosInches = TunerConstants.BackLeft.LocationY;
        kBackRightDriveMotorId = TunerConstants.BackRight.DriveMotorId;
        kBackRightSteerMotorId = TunerConstants.BackRight.SteerMotorId;
        kBackRightEncoderId = TunerConstants.BackRight.CANcoderId;
        kBackRightDriveCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kBackRightSteerCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kBackRightEncoderCanbus = TunerConstants.DrivetrainConstants.CANBusName;
        kBackRightDriveType = "kraken";
        kBackRightSteerType = "kraken";
        kBackRightEncoderType = "cancoder";
        kBackRightEncoderOffset =
            -Units.rotationsToRadians(TunerConstants.BackRight.CANcoderOffset);
        kBackRightDriveInvert = TunerConstants.BackRight.DriveMotorInverted;
        kBackRightSteerInvert = TunerConstants.BackRight.SteerMotorInverted;
        kBackRightEncoderInvert = false;
        kBackRightXPosInches = TunerConstants.BackRight.LocationX;
        kBackRightYPosInches = TunerConstants.BackRight.LocationY;
        // NOTE: The PIDF values from TunerConstants.java make WPILib/AK implemention go crazy
        //       These values are from the AK example sketches
        kDriveP = 0.05;
        kDriveI = 0.0;
        kDriveD = 0.0;
        kDriveF = 0.13;
        kDriveIZ = 0.0;
        kSteerP = 7.0;
        kSteerI = 0.0;
        kSteerD = 0.0;
        kSteerF = 0.0;
        kSteerIZ = 0.0;
        break;

      case YAGSL:
        kCoupleRatio = YagslConstants.kCoupleRatio;
        kDriveGearRatio = YagslConstants.kDriveGearRatio;
        kSteerGearRatio = YagslConstants.kSteerGearRatio;
        kWheelRadiusInches = YagslConstants.kWheelRadiusInches;
        kCANbusName = YagslConstants.kCANbusName;
        kPigeonId = YagslConstants.kPigeonId;
        kSteerInertia = YagslConstants.kSteerInertia;
        kDriveInertia = YagslConstants.kDriveInertia;
        kSteerFrictionVoltage = YagslConstants.kSteerFrictionVoltage;
        kDriveFrictionVoltage = YagslConstants.kDriveFrictionVoltage;
        kSteerCurrentLimit = YagslConstants.kSteerCurrentLimit;
        kDriveCurrentLimit = YagslConstants.kDriveCurrentLimit;
        kOptimalVoltage = YagslConstants.kOptimalVoltage;
        kFrontLeftDriveMotorId = YagslConstants.kFrontLeftDriveMotorId;
        kFrontLeftSteerMotorId = YagslConstants.kFrontLeftSteerMotorId;
        kFrontLeftEncoderId = YagslConstants.kFrontLeftEncoderId;
        kFrontLeftDriveCanbus = YagslConstants.kFrontLeftDriveCanbus;
        kFrontLeftSteerCanbus = YagslConstants.kFrontLeftSteerCanbus;
        kFrontLeftEncoderCanbus = YagslConstants.kFrontLeftEncoderCanbus;
        kFrontLeftDriveType = YagslConstants.kFrontLeftDriveType.toLowerCase();
        kFrontLeftSteerType = YagslConstants.kFrontLeftSteerType.toLowerCase();
        kFrontLeftEncoderType = YagslConstants.kFrontLeftEncoderType.toLowerCase();
        kFrontLeftEncoderOffset = Units.degreesToRadians(YagslConstants.kFrontLeftEncoderOffset);
        kFrontLeftDriveInvert = YagslConstants.kFrontLeftDriveInvert;
        kFrontLeftSteerInvert = YagslConstants.kFrontLeftSteerInvert;
        kFrontLeftEncoderInvert = YagslConstants.kFrontLeftEncoderInvert;
        kFrontLeftXPosInches = YagslConstants.kFrontLeftXPosInches;
        kFrontLeftYPosInches = YagslConstants.kFrontLeftYPosInches;
        kFrontRightDriveMotorId = YagslConstants.kFrontRightDriveMotorId;
        kFrontRightSteerMotorId = YagslConstants.kFrontRightSteerMotorId;
        kFrontRightEncoderId = YagslConstants.kFrontRightEncoderId;
        kFrontRightDriveCanbus = YagslConstants.kFrontRightDriveCanbus;
        kFrontRightSteerCanbus = YagslConstants.kFrontRightSteerCanbus;
        kFrontRightEncoderCanbus = YagslConstants.kFrontRightEncoderCanbus;
        kFrontRightDriveType = YagslConstants.kFrontRightDriveType.toLowerCase();
        kFrontRightSteerType = YagslConstants.kFrontRightSteerType.toLowerCase();
        kFrontRightEncoderType = YagslConstants.kFrontRightEncoderType.toLowerCase();
        kFrontRightEncoderOffset = Units.degreesToRadians(YagslConstants.kFrontRightEncoderOffset);
        kFrontRightDriveInvert = YagslConstants.kFrontRightDriveInvert;
        kFrontRightSteerInvert = YagslConstants.kFrontRightSteerInvert;
        kFrontRightEncoderInvert = YagslConstants.kFrontRightEncoderInvert;
        kFrontRightXPosInches = YagslConstants.kFrontRightXPosInches;
        kFrontRightYPosInches = YagslConstants.kFrontRightYPosInches;
        kBackLeftDriveMotorId = YagslConstants.kBackLeftDriveMotorId;
        kBackLeftSteerMotorId = YagslConstants.kBackLeftSteerMotorId;
        kBackLeftEncoderId = YagslConstants.kBackLeftEncoderId;
        kBackLeftDriveCanbus = YagslConstants.kBackLeftDriveCanbus;
        kBackLeftSteerCanbus = YagslConstants.kBackLeftSteerCanbus;
        kBackLeftEncoderCanbus = YagslConstants.kBackLeftEncoderCanbus;
        kBackLeftDriveType = YagslConstants.kBackLeftDriveType.toLowerCase();
        kBackLeftSteerType = YagslConstants.kBackLeftSteerType.toLowerCase();
        kBackLeftEncoderType = YagslConstants.kBackLeftEncoderType.toLowerCase();
        kBackLeftEncoderOffset = Units.degreesToRadians(YagslConstants.kBackLeftEncoderOffset);
        kBackLeftDriveInvert = YagslConstants.kBackLeftDriveInvert;
        kBackLeftSteerInvert = YagslConstants.kBackLeftSteerInvert;
        kBackLeftEncoderInvert = YagslConstants.kBackLeftEncoderInvert;
        kBackLeftXPosInches = YagslConstants.kBackLeftXPosInches;
        kBackLeftYPosInches = YagslConstants.kBackLeftYPosInches;
        kBackRightDriveMotorId = YagslConstants.kBackRightDriveMotorId;
        kBackRightSteerMotorId = YagslConstants.kBackRightSteerMotorId;
        kBackRightEncoderId = YagslConstants.kBackRightEncoderId;
        kBackRightDriveCanbus = YagslConstants.kBackRightDriveCanbus;
        kBackRightSteerCanbus = YagslConstants.kBackRightSteerCanbus;
        kBackRightEncoderCanbus = YagslConstants.kBackRightEncoderCanbus;
        kBackRightDriveType = YagslConstants.kBackRightDriveType.toLowerCase();
        kBackRightSteerType = YagslConstants.kBackRightSteerType.toLowerCase();
        kBackRightEncoderType = YagslConstants.kBackRightEncoderType.toLowerCase();
        kBackRightEncoderOffset = Units.degreesToRadians(YagslConstants.kBackRightEncoderOffset);
        kBackRightDriveInvert = YagslConstants.kBackRightDriveInvert;
        kBackRightSteerInvert = YagslConstants.kBackRightSteerInvert;
        kBackRightEncoderInvert = YagslConstants.kBackRightEncoderInvert;
        kBackRightXPosInches = YagslConstants.kBackRightXPosInches;
        kBackRightYPosInches = YagslConstants.kBackRightYPosInches;
        kDriveP = YagslConstants.kDriveP;
        kDriveI = YagslConstants.kDriveI;
        kDriveD = YagslConstants.kDriveD;
        kDriveF = YagslConstants.kDriveF;
        kDriveIZ = YagslConstants.kDriveIZ;
        kSteerP = YagslConstants.kSteerP;
        kSteerI = YagslConstants.kSteerI;
        kSteerD = YagslConstants.kSteerD;
        kSteerF = YagslConstants.kSteerF;
        kSteerIZ = YagslConstants.kSteerIZ;
        break;

      default:
        throw new RuntimeException("Invalid Swerve Drive Type");
    }
  }

  // Computed quantities
  public static final double kDriveBaseRadiusInches =
      Math.hypot(kFrontLeftXPosInches, kFrontLeftYPosInches);
  public static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);

  // Stuff to deal with from AK25's Spark Swerve Template
  public static final double maxSpeedMetersPerSec = 4.8;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(26.5);
  public static final double wheelBase = Units.inchesToMeters(26.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}
