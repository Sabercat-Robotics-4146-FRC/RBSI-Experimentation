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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
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
    SIM, // SIM == Running a physics simulator
    REPLAY // REPLAY == Replaying from a log file
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

  /** Get the current mode for robot operation, based on robot type */
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

  /***************************************************************************/
  /* The remainder of this file contains physical and/or software constants for the various subsystems of the robot */

  /** General Constants **************************************************** */
  private static RobotType robotType = getRobot();

  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;

  /** Physical Constants for Robot Operation ************ */
  public static final class PhysicalConstants {
    public static final double kRobotMass = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter kChassis =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), PhysicalConstants.kRobotMass);
    public static final double kLoopTime = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double kMaxSpeed = Units.feetToMeters(14.5);
    // Maximum speed of the robot in meters per second, used to limit acceleration.
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
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  /** Drive Base Constants ************************************************* */
  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  /** Operator Constants *************************************************** */
  public static class OperatorConstants {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  /** AprilTag Field Layout ************************************************ */
  /* SEASON SPECIFIC! */
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
                  Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", name + ".json"));
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
              "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
        }
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }
}
