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

package frc.robot;

import static frc.robot.util.RBSIEnum.*;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /***************************************************************************/
  /**
   * Define the various multiple robots that use this same code (e.g., COMPBOT, DEVBOT, SIMBOT,
   * etc.) and the operating modes of the code (REAL, SIM, or REPLAY)
   */
  private static RobotType robotType = RobotType.COMPBOT;

  // Define swerve, auto, and vision types being used
  // NOTE: Only PHOENIX6 swerve base has been tested at this point!!!
  //       If you have a swerve base with non-CTRE compoments, use YAGSL
  //       under strict caveat emptor -- and submit any error and bugfixes
  //       via GitHub issues.
  private static SwerveType swerveType = SwerveType.PHOENIX6;
  private static AutoType autoType = AutoType.PATHPLANNER;
  private static VisionType visionType = VisionType.NONE;

  /** Checks whether the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType);
      System.exit(1);
    }
  }

  /** Disable the Hardware Abstraction Layer, if requested */
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /***************************************************************************/
  /* The remainder of this file contains physical and/or software constants for the various subsystems of the robot */

  /** General Constants **************************************************** */
  public static final double loopPeriodSecs = 0.02;

  public static final boolean tuningMode = false;

  /** Physical Constants for Robot Operation ******************************* */
  public static final class PhysicalConstants {

    public static final double kRobotMass = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter kChassis =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMass);
    public static final double kLoopTime = 0.13; // s, 20ms + 110ms sprk max velocity lag
  }

  /** Power Distribution Constants ********************************** */
  public static final class PowerConstants {

    // Set this to either kRev or kCTRE for the type of Power Distribution Module
    public static final ModuleType kPowerModule = ModuleType.kRev;
    // Current Limits
    public static final double kTotalMaxCurrent = 120.;
    public static final double kMotorPortMaxCurrent = 40.;
    public static final double kSmallPortMaxCurrent = 20.;
  }

  /** Accelerometer Constants ********************************************** */
  public static class AccelerometerConstants {

    // Insert here the orientation (CCW == +) of the Rio and IMU from the robot
    // An angle of "0." means the x-y-z markings on the device match the robot's intrinsic reference
    //   frame.
    // NOTE: It is assumed that both the Rio and the IMU are mounted such that +Z is UP
    public static final Rotation2d kRioOrientation =
        switch (getRobot()) {
          case COMPBOT -> Rotation2d.fromDegrees(0.);
          case DEVBOT -> Rotation2d.fromDegrees(0.);
          default -> Rotation2d.fromDegrees(0.);
        };
    // IMU can be one of Pigeon2 or NavX
    public static final Rotation2d kIMUOrientation =
        switch (getRobot()) {
          case COMPBOT -> Rotation2d.fromDegrees(0.);
          case DEVBOT -> Rotation2d.fromDegrees(0.);
          default -> Rotation2d.fromDegrees(0.);
        };
  }

  /** Drive Base Constants ************************************************* */
  public static final class DrivebaseConstants {

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    // NOTE: If using SwerveType.PHOENIX6, adjust this in the Phoenix X Tuner Swerve Generator
    public static final LinearVelocity kMaxLinearSpeed = TunerConstants.kSpeedAt12Volts;
    //       Otherwise, set the maximum linear speed here
    // public static final double kMaxLinearSpeed = 5.21;

    // Set 3/4 of a rotation per second as the max angular velocity
    public static final double kMaxAngularSpeed = 1.5 * Math.PI;

    // Maximum chassis accelerations desired for robot motion  -- metric / radians
    public static final double kMaxLinearAccel = 4.0; // m/s/s
    public static final double kMaxAngularAccel = Units.degreesToRadians(720); // deg/s/s

    // Hold time on motor brakes when disabled
    public static final double kWheelLockTime = 10; // seconds

    // SysID characterization constants
    public static final double kMaxV = 12.0; // Max volts
    public static final double kDelay = 3.0; // seconds
    public static final double kQuasiTimeout = 5.0; // seconds
    public static final double kDynamicTimeout = 3.0; // seconds
  }

  /** Example Flywheel Mechanism Constants ********************************* */
  public static final class FlywheelConstants {

    // Mechanism motor gear ratio
    public static final double kFlywheelGearRatio = 1.5;

    // MODE == REAL / REPLAY
    // Feedforward constants
    public static final double kStaticGainReal = 0.1;
    public static final double kVelocityGainReal = 0.05;
    // Feedback (PID) constants
    public static final double kPReal = 1.0;
    public static final double kIReal = 0.0;
    public static final double kDReal = 0.0;

    // MODE == SIM
    // Feedforward constants
    public static final double kStaticGainSim = 0.0;
    public static final double kVelocityGainSim = 0.03;
    // Feedback (PID) constants
    public static final double kPSim = 1.0;
    public static final double kISim = 0.0;
    public static final double kDSim = 0.0;
  }

  /** Operator Constants *************************************************** */
  public static class OperatorConstants {

    // Joystick Deadband
    public static final double kLeftDeadband = 0.1;
    public static final double kRightDeadband = 0.1;
    public static final double kTurnConstant = 6;
  }

  /** Autonomous Action Constants ****************************************** */
  public static final class AutonConstants {

    // Translation PID constants
    public static final PIDConstants kAutoTranslatePID = new PIDConstants(0.7, 0, 0);
    // Rotation PID constants
    public static final PIDConstants kAutoAnglePID = new PIDConstants(0.4, 0, 0.01);
  }

  /** Choreo Autonomous Action Constants *********************************** */
  public static final class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  /** Vision Constants (Assuming PhotonVision) ***************************** */
  public static class VisionConstants {

    // AprilTag Identification Constants
    public static final double kAmbiguityThreshold = 0.4;
    public static final double kTargetLogTimeSecs = 0.1;
    public static final double kFieldBorderMargin = 0.5;
    public static final double kZMargin = 0.75;
    public static final double kXYZStdDevCoefficient = 0.005;
    public static final double kThetaStdDevCoefficient = 0.01;
  }

  /** AprilTag Field Layout ************************************************ */
  /* SEASON SPECIFIC! -- This section is for 2024 (Crescendo) */
  // NOTE: This section will be updated to 2025 "Reefscape" following kickoff
  public static class AprilTagConstants {

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    @Getter
    public enum AprilTagLayoutType {
      OFFICIAL("2024-official"),
      SPEAKERS_ONLY("2024-speakers"),
      AMPS_ONLY("2024-amps"),
      WPI("2024-wpi");

      private AprilTagLayoutType(String name) {
        if (Constants.disableHAL) {
          layout = null;
        } else {
          try {
            layout =
                new AprilTagFieldLayout(
                    Path.of(
                        Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
          } catch (IOException e) {
            throw new RuntimeException(e);
          }
        }
        if (layout == null) {
          layoutString = "";
        } else {
          try {
            layoutString = new ObjectMapper().writeValueAsString(layout);
          } catch (JsonProcessingException e) {
            throw new RuntimeException(
                "Failed to serialize AprilTag layout JSON " + toString() + "for PhotonVision");
          }
        }
      }

      private final AprilTagFieldLayout layout;
      private final String layoutString;
    }
  }

  /** Deploy Directoy Location Constants *********************************** */
  public static final class DeployConstants {
    public static final String apriltagDir = "apriltags";
    public static final String choreoDir = "choreo";
    public static final String pathplannerDir = "pathplanner";
    public static final String yagslDir = "swerve";
  }

  /***************************************************************************/
  /** Getter functions -- do not modify ************************************ */
  /** Get the current robot */
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  /** Get the current robot mode */
  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  /** Get the current swerve drive type */
  public static SwerveType getSwerveType() {
    return swerveType;
  }

  /** Get the current autonomous path planning type */
  public static AutoType getAutoType() {
    return autoType;
  }

  /** Get the current autonomous path planning type */
  public static VisionType getVisionType() {
    return visionType;
  }
}
