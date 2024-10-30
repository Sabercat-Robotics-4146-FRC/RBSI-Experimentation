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

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
  private static RobotType robotType = RobotType.SIMBOT;

  private static SwerveType swerveType = SwerveType.PHOENIX6;

  public static boolean disableHAL = false;

  /** Enumerate the robot types (add additional bots here) */
  public static enum RobotType {
    DEVBOT, // Development / Alpha / Practice Bot
    COMPBOT, // Competition robot
    SIMBOT // Simulated robot
  }

  /** Enumerate the robot operation modes */
  public static enum Mode {
    REAL, // REAL == Running on a real robot
    REPLAY, // REPLAY == Replaying from a log file
    SIM // SIM == Running a physics simulator
  }

  /** Get the current robot */
  public static RobotType getRobot() {
    if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  /** Get the current mode */
  public static Mode getMode() {
    return switch (robotType) {
      case DEVBOT, COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  /** Disable the Hardware Abstraction Layer, if requested */
  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType);
      System.exit(1);
    }
  }

  /** Enumerate the supported swerve drive types */
  public static enum SwerveType {
    PHOENIX6, // The all-CTRE Phoenix6 swerve library
    YAGSL // The generic YAGSL swerve library (not supported at this time)
  }

  /** Get the current swerve drive type */
  public static SwerveType getSwerve() {
    return swerveType;
  }

  /***************************************************************************/
  /* The remainder of this file contains physical and/or software constants for the various subsystems of the robot */

  /** General Constants **************************************************** */
  public static final double loopPeriodSecs = 0.02;

  public static final boolean tuningMode = false;

  /** Accelerometer Constants ********************************************** */
  public static class AccelerometerConstants {
    // Insert here the orientation (CCW == +) of the Rio and Pigeon2 from the robot
    // An angle of "0." means the x-y-z markings on the device match the robot's intrinsic reference
    //   frame.
    // NOTE: It is assumed that both the Rio and the Pigeon are mounted such that +Z is UP
    public static final Rotation2d kRioOrientation =
        switch (getRobot()) {
          case COMPBOT -> Rotation2d.fromDegrees(0.);
          case DEVBOT -> Rotation2d.fromDegrees(0.);
          default -> Rotation2d.fromDegrees(0.);
        };
    public static final Rotation2d kPigeonOrientation =
        switch (getRobot()) {
          case COMPBOT -> Rotation2d.fromDegrees(0.);
          case DEVBOT -> Rotation2d.fromDegrees(0.);
          default -> Rotation2d.fromDegrees(0.);
        };
  }

  /** Physical Constants for Robot Operation ******************************* */
  public static final class PhysicalConstants {
    public static final double kRobotMass = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter kChassis =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), PhysicalConstants.kRobotMass);
    public static final double kLoopTime = 0.13; // s, 20ms + 110ms sprk max velocity lag
  }

  /** Power Distribution Module Constants ********************************** */
  public static final class PowerConstants {

    // Set this to either kRev or kCTRE for the type of Power Distribution Module
    public static final ModuleType kPowerModule = ModuleType.kRev;

    // Current Limits
    public static final double kTotalMaxCurrent = 120.;
    public static final double kMotorPortMaxCurrent = 40.;
    public static final double kSmallPortMaxCurrent = 20.;

    // The Power Distribution Module ports into which the DRIVE motors are plugged
    public static final int[] kDrivePowerPorts = {0, 2, 4, 6};
    // The Power Distribution Module ports into which the STEER motors are plugged
    public static final int[] kSteerPowerPorts = {1, 3, 5, 7};
    // Add additional subsystem port enumerations here for combined monitoring
    // Example:
    // public static final int[] kElevatorPowerPorts = {9, 10};
  }

  /** Autonomous Action Constants ****************************************** */
  public static final class AutonConstants {

    // Translation PID constants
    public static final PIDConstants kAutoTranslatePID = new PIDConstants(0.7, 0, 0);
    // Rotation PID constants
    public static final PIDConstants kAutoAnglePID = new PIDConstants(0.4, 0, 0.01);
  }

  /** Drive Base Constants ************************************************* */
  public static final class DrivebaseConstants {

    // Physical size of the drive base
    private static final double kTrackWidthX = Units.inchesToMeters(20.75);
    private static final double kTrackWidthY = Units.inchesToMeters(20.75);
    public static final double kDriveBaseRadius =
        Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);

    // CTRE-based maximum robot speeds (adjustable in Phoenix X Tuner Swerve Generator)
    // kSpeedAt12VoltsMps desired top speed
    public static final double kMaxLinearSpeed = TunerConstants.kSpeedAt12VoltsMps;
    // 3/4 of a rotation per second max angular velocity
    public static final double kMaxAngularSpeed = 1.5 * Math.PI;

    // Maximum chassis accelerations desired for robot motion  -- metric / radians
    public static final double kMaxLinearAccel = 4.0; // m/s/s
    public static final double kMaxAngularAccel = Units.degreesToRadians(720); // deg/s/s

    // Wheel radius
    public static final double kWheelRadius = Units.inchesToMeters(2.0);

    // ** Gear ratios for SDS MK4i L2, adjust as necessary **
    public static final double kDriveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double kTurnGearRatio = 150.0 / 7.0;

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
    public static final double kLeftXDeadband = 0.1;
    public static final double kLeftYDeadband = 0.1;
    public static final double kRightXDeadband = 0.1;
    public static final double kRightYDeadband = 0.1;
    public static final double kTurnConstant = 6;
  }

  /** AprilTag Field Layout ************************************************ */
  /* SEASON SPECIFIC! */
  public static class AprilTagConstants {

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

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
}
