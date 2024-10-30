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
//
// NOTE: This module based on the CTRE Phoenix6 examples
//       https://github.com/CrossTheRoadElec/Phoenix6-Examples

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DrivebaseConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class SwerveSubsystem extends SwerveDrivetrain implements Subsystem {
  /** CTRE constants */
  private static final double kSimLoopPeriod = 0.005; // 5 ms

  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /** PhotonVision class to keep an accurate odometry */
  private Vision vision;

  /** Enable vision odometry updates while driving */
  private final boolean visionDriveTest = false;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  /* Use one of these sysidroutines for your particular test */
  private SysIdRoutine SysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(TranslationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine SysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(RotationCharacterization.withVolts(volts)), null, this));
  private final SysIdRoutine SysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(7),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(SteerCharacterization.withVolts(volts)), null, this));

  /* Change this to the sysid routine you want to test */
  private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

  public SwerveSubsystem(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    configurePathPlanner();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public SwerveSubsystem(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    configurePathPlanner();
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  private void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds) ->
            this.setControl(
                AutoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(
            new PIDConstants(10, 0, 0),
            new PIDConstants(10, 0, 0),
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().orElse(Alliance.Blue)
                == Alliance
                    .Red, // Assume the path needs to be flipped for Red vs Blue, this is normally
        // the case
        this); // Subsystem for requirements
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  /*
   * Both the sysid commands are specific to one particular sysid routine, change
   * which one you're trying to characterize
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return RoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return RoutineToApply.dynamic(direction);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /** Periodic function -- update odometry and log everything */
  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
    /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
    /* This allows us to correct the perspective in case the robot code restarts mid-match */
    /* Otherwise, only check and apply the operator perspective if the DS is disabled */
    /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RedAlliancePerspectiveRotation
                        : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
    }

    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest) {
      vision.updatePoseEstimation(this);
    }

    /** Log Telemetry Data to AdvantageKit */
    // NOTE: Phoenix6 telemetry NOT in SwerveDriveTelemetry!!!
    Logger.recordOutput("SwerveDive/Telemetry/moduleCount", SwerveDriveTelemetry.moduleCount);
    Logger.recordOutput("SwerveDive/Telemetry/wheelLocations", SwerveDriveTelemetry.wheelLocations);
    Logger.recordOutput("SwerveDive/Telemetry/measuredStates", SwerveDriveTelemetry.measuredStates);
    Logger.recordOutput("SwerveDive/Telemetry/desiredStates", SwerveDriveTelemetry.desiredStates);
    Logger.recordOutput("SwerveDive/Telemetry/robotRotation", SwerveDriveTelemetry.robotRotation);
    Logger.recordOutput("SwerveDive/Telemetry/maxSpeed", SwerveDriveTelemetry.maxSpeed);
    Logger.recordOutput("SwerveDive/Telemetry/rotationUnit", SwerveDriveTelemetry.rotationUnit);
    Logger.recordOutput("SwerveDive/Telemetry/sizeLeftRight", SwerveDriveTelemetry.sizeLeftRight);
    Logger.recordOutput("SwerveDive/Telemetry/sizeFrontBack", SwerveDriveTelemetry.sizeFrontBack);
    Logger.recordOutput(
        "SwerveDive/Telemetry/forwardDirection", SwerveDriveTelemetry.forwardDirection);
    Logger.recordOutput(
        "SwerveDive/Telemetry/maxAngularVelocity", SwerveDriveTelemetry.maxAngularVelocity);
    Logger.recordOutput(
        "SwerveDive/Telemetry/measuredChassisSpeeds", SwerveDriveTelemetry.measuredChassisSpeeds);
    Logger.recordOutput(
        "SwerveDive/Telemetry/desiredChassisSpeeds", SwerveDriveTelemetry.desiredChassisSpeeds);

    /** Log Swerve Drive States to AdvantageKit */
    getModuleStates();
    getDesiredStates();
    Logger.recordOutput(
        "SwerveDive/States/RobotRotation",
        SwerveDriveTelemetry.rotationUnit == "degrees"
            ? Rotation2d.fromDegrees(SwerveDriveTelemetry.robotRotation)
            : Rotation2d.fromRadians(SwerveDriveTelemetry.robotRotation));
  }

  /************************************************************************* */
  /* COMMAND SECTION -- Drivebase-only Commands */

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            MAX_LINEAR_SPEED, MAX_LINEAR_ACCEL, MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCEL);

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before
        // attempting to rotate.
        );
  }

  /************************************************************************* */
  /* UTILITY SECTION -- Utility methods */

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  @AutoLogOutput(key = "Odometry/RobotPose")
  public Pose2d getPose() {
    return getState().Pose;
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    if (brake) {
      this.configNeutralMode(NeutralModeValue.Brake);
    } else {
      this.configNeutralMode(NeutralModeValue.Coast);
    }
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveDive/States/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] =
          new SwerveModuleState(
              SwerveDriveTelemetry.measuredStates[(i * 2) + 1],
              Rotation2d.fromDegrees(SwerveDriveTelemetry.measuredStates[i * 2]));
    }
    return states;
  }

  /** Returns the desired states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveDive/States/Desired")
  private SwerveModuleState[] getDesiredStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] =
          new SwerveModuleState(
              SwerveDriveTelemetry.desiredStates[(i * 2) + 1],
              Rotation2d.fromDegrees(SwerveDriveTelemetry.desiredStates[i * 2]));
    }
    return states;
  }
}
