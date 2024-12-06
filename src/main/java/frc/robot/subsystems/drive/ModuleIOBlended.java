// Copyright (c) 2024 Az-FIRST
// http://github.com/AZ-First
// Copyright 2024 FRC 2486
// https://github.com/Coconuts2486-FRC
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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Module IO implementation for blended TalonFX drive motor controller, SparkMax turn motor
 * controller (NEO or NEO 550), and CANcoder.
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOBlended implements ModuleIO {
  // CAN Devices
  private final TalonFX driveTalon;
  private final SparkBase turnSparkMax;
  private final CANcoder cancoder;

  // Drive telemetry information
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  // Steer telemetry information
  private final StatusSignal<Double> turnAbsolutePosition;
  private final RelativeEncoder turnRelativeEncoder;

  private final boolean isTurnMotorInverted;
  private final Rotation2d absoluteEncoderOffset;

  /*
   * Blended Module I/O, using Falcon drive and NEO steer motors
   * Based on the ModuleIOTalonFX module, with the SparkMax components
   * added in appropriately.
   */
  public ModuleIOBlended(int index) {
    switch (index) {
      case 0:
        // Front Left
        driveTalon = new TalonFX(kFrontLeftDriveMotorId, kFrontLeftDriveCanbus);
        turnSparkMax = new SparkBase(kFrontLeftSteerMotorId, MotorType.kBrushless);
        cancoder = new CANcoder(kFrontLeftEncoderId, kFrontLeftEncoderCanbus);
        absoluteEncoderOffset = new Rotation2d(kFrontLeftEncoderOffset);
        isTurnMotorInverted = kFrontLeftSteerInvert;
        break;

      case 1:
        // Front Right
        driveTalon = new TalonFX(kFrontRightDriveMotorId, kFrontRightDriveCanbus);
        turnSparkMax = new SparkBase(kFrontRightSteerMotorId, MotorType.kBrushless);
        cancoder = new CANcoder(kFrontRightEncoderId, kFrontRightEncoderCanbus);
        absoluteEncoderOffset = new Rotation2d(kFrontRightEncoderOffset);
        isTurnMotorInverted = kFrontRightSteerInvert;
        break;

      case 2:
        // Back Left
        driveTalon = new TalonFX(kBackLeftDriveMotorId, kBackLeftDriveCanbus);
        turnSparkMax = new SparkBase(kBackLeftSteerMotorId, MotorType.kBrushless);
        cancoder = new CANcoder(kBackLeftEncoderId, kBackLeftEncoderCanbus);
        absoluteEncoderOffset = new Rotation2d(kBackLeftEncoderOffset);
        isTurnMotorInverted = kBackLeftSteerInvert;
        break;

      case 3:
        // Back Right
        driveTalon = new TalonFX(kBackRightDriveMotorId, kBackRightDriveCanbus);
        turnSparkMax = new SparkBase(kBackRightSteerMotorId, MotorType.kBrushless);
        cancoder = new CANcoder(kBackRightEncoderId, kBackRightEncoderCanbus);
        absoluteEncoderOffset = new Rotation2d(kBackRightEncoderOffset);
        isTurnMotorInverted = kBackRightSteerInvert;
        break;

      default:
        throw new RuntimeException("Invalid module index");
    }

    // Drive Configuration
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = kDriveCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = kDriveCurrentLimit * 0.6;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    // Steer Configuration
    turnSparkMax.restoreFactoryDefaults();
    turnSparkMax.setCANTimeout(250);
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnSparkMax.setInverted(isTurnMotorInverted);
    turnSparkMax.setSmartCurrentLimit((int) kSteerCurrentLimit);
    turnSparkMax.enableVoltageCompensation(kOptimalVoltage);
    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);
    turnSparkMax.setCANTimeout(0);

    // CANCoder Configuration
    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getSupplyCurrent();
    turnAbsolutePosition = cancoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, drivePosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnAbsolutePosition);
    driveTalon.optimizeBusUtilization();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, turnAbsolutePosition);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / kDriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / kDriveGearRatio;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
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
    driveTalon.setControl(new VoltageOut(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
