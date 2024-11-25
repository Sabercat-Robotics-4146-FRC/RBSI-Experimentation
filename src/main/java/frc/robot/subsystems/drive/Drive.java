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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.YagslConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator m_PoseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  // Constructor
  public Drive() {
    switch (Constants.getSwerveType()) {
      case PHOENIX6:
        // This one is easy because it's all CTRE all the time
        gyroIO = new GyroIOPigeon2();
        for (int i = 0; i < 4; i++) {
          modules[i] = new Module(new ModuleIOTalonFX(i), i);
        }
        break;

      case YAGSL:
        // First, choose the Gyro
        switch (YagslConstants.swerveDriveJson.imu.type) {
          case "pigeon2":
            gyroIO = new GyroIOPigeon2();
            break;
          case "navx":
          case "navx_spi":
            gyroIO = new GyroIONavX();
            break;
          default:
            throw new RuntimeException("Invalid IMU type");
        }
        // Then parse the module(s)
        Byte modType = parseModuleType();
        for (int i = 0; i < 4; i++) {
          switch (modType) {
            case 0b00000000: // ALL-CTRE
              modules[i] = new Module(new ModuleIOTalonFX(i), i);
              break;
            case 0b00010000: // Blended Talon Drive / NEO Steer
              modules[i] = new Module(new ModuleIOBlended(i), i);
              break;
            case 0b01010000: // NEO motors + CANcoder
              modules[i] = new Module(new ModuleIOSparkCANcoder(i), i);
              break;
            case 0b01010100: // NEO motors + analog encoder
              modules[i] = new Module(new ModuleIOSparkMax(i), i);
              break;
            default:
              throw new RuntimeException("Invalid swerve module combination");
          }
        }

      default:
        throw new RuntimeException("Invalid Swerve Drive Type");
    }

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            DrivebaseConstants.kMaxLinearSpeed,
            Units.inchesToMeters(kDriveBaseRadius),
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    m_PoseEstimator.update(rawGyroRotation, modulePositions);
  }

  /**
   * Sets the swerve drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    {
      for (Module swerveModule : modules) {
        swerveModule.setBrakeMode(brake);
      }
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DrivebaseConstants.kMaxLinearSpeed);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Re-zero the gyro at the present heading */
  public void zero() {
    gyroIO.zero();
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_PoseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    m_PoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    m_PoseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DrivebaseConstants.kMaxLinearSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return DrivebaseConstants.kMaxAngularSpeed;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.kFrontLeftXPosInches, DriveConstants.kFrontLeftYPosInches),
      new Translation2d(DriveConstants.kFrontRightXPosInches, DriveConstants.kFrontRightYPosInches),
      new Translation2d(DriveConstants.kBackLeftXPosInches, DriveConstants.kBackLeftYPosInches),
      new Translation2d(DriveConstants.kBackRightXPosInches, DriveConstants.kBackRightYPosInches)
    };
  }

  public <T> T getGyro() {
    return gyroIO.getGyro();
  }

  /**
   * Parse the module type given the type information for the FL module
   *
   * @return Byte The module type as bits in a byte.
   */
  private Byte parseModuleType() {
    // NOTE: This assumes all 4 modules have the same arrangement!
    Byte b_drive; // [x,x,-,-,-,-,-,-]
    Byte b_steer; // [-,-,x,x,-,-,-,-]
    Byte b_encoder; // [-,-,-,-,x,x,-,-]
    switch (kFrontLeftDriveType) {
      case "falcon":
      case "kraken":
      case "talonfx":
        // CTRE Drive Motor
        b_drive = 0b00;
        break;
      case "sparkmax":
      case "sparkflex":
        // REV Drive Motor
        b_drive = 0b01;
        break;
      default:
        throw new RuntimeException("Invalid drive motor type");
    }
    switch (kFrontLeftSteerType) {
      case "falcon":
      case "kraken":
      case "talonfx":
        // CTRE Drive Motor
        b_steer = 0b00;
        break;
      case "sparkmax":
      case "sparkflex":
        // REV Drive Motor
        b_steer = 0b01;
        break;
      default:
        throw new RuntimeException("Invalid steer motor type");
    }
    switch (kFrontLeftEncoderType) {
      case "cancoder":
        // CTRE CANcoder
        b_encoder = 0b00;
        break;
      case "analog":
        // Analog Encoder
        b_encoder = 0b01;
        break;
      default:
        throw new RuntimeException("Invalid swerve encoder type");
    }
    return (byte) (0b00000000 | b_drive << 6 | b_steer << 4 | b_encoder << 2);
  }
}
