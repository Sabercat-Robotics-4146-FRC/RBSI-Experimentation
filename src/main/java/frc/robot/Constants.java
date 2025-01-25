// Copyright (c) 2024-2025 Az-FIRST
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
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.AprilTagConstants.AprilTagLayoutType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.RBSIEnum.AutoType;
import frc.robot.util.RBSIEnum.CTREPro;
import frc.robot.util.RBSIEnum.Mode;
import frc.robot.util.RBSIEnum.MotorIdleMode;
import frc.robot.util.RBSIEnum.RobotType;
import frc.robot.util.RBSIEnum.SwerveType;
import frc.robot.util.RBSIEnum.VisionType;
import frc.robot.util.RobotDeviceId;
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
  private static SwerveType swerveType = SwerveType.PHOENIX6; // PHOENIX6, YAGSL
  private static CTREPro phoenixPro = CTREPro.LICENSED; // LICENSED, UNLICENSED
  private static AutoType autoType = AutoType.PATHPLANNER; // PATHPLANNER, CHOREO
  private static VisionType visionType = VisionType.PHOTON; // PHOTON, LIMELIGHT, NONE

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

    public static final double kRobotMassKg = Units.lbsToKilograms(100.);
    public static final Matter kChassis =
        new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMassKg);
    // Robot moment of intertial; this can be obtained from a CAD model of your drivetrain. Usually,
    // this is between 3 and 8 kg*m^2.
    public static final double kRobotMOI = 6.8;

    // Wheel coefficient of friction
    public static final double kWheelCOF = 1.2;
  }

  /** Power Distribution Constants ********************************** */
  public static final class PowerConstants {

    // Current Limits
    public static final double kTotalMaxCurrent = 120.;
    public static final double kMotorPortMaxCurrent = 40.;
    public static final double kSmallPortMaxCurrent = 20.;
  }

  /** Drive Base Constants ************************************************* */
  public static final class DrivebaseConstants {

    // Theoretical free speed (m/s) at 12v applied output;
    // IMPORTANT: Follow the AdvantageKit instructions for measuring the ACTUAL maximum linear speed
    // of YOUR ROBOT, and replace the estimate here with your measured value!
    public static final double kMaxLinearSpeed = Units.feetToMeters(18);

    // Set 3/4 of a rotation per second as the max angular velocity (radians/sec)
    public static final double kMaxAngularSpeed = 1.5 * Math.PI;

    // Maximum chassis accelerations desired for robot motion  -- metric / radians
    // TODO: Compute the maximum linear acceleration given the PHYSICS of the ROBOT!
    public static final double kMaxLinearAccel = 4.0; // m/s/s
    public static final double kMaxAngularAccel = Units.degreesToRadians(720);

    // Hold time on motor brakes when disabled
    public static final double kWheelLockTime = 10; // seconds

    // SysID characterization constants
    public static final double kMaxV = 12.0; // Max volts
    public static final double kDelay = 3.0; // seconds
    public static final double kQuasiTimeout = 5.0; // seconds
    public static final double kDynamicTimeout = 3.0; // seconds

    // Not sure what to do with these, yet...
    // kDriveF = 0.13;
    // kDriveIZ = 0.0;
    // kSteerF = 0.0;
    // kSteerIZ = 0.0;
  }

  /** Example Flywheel Mechanism Constants ********************************* */
  public static final class FlywheelConstants {

    // Mechanism idle mode
    public static final MotorIdleMode kFlywheelIdleMode = MotorIdleMode.COAST; // BRAKE, COAST

    // Mechanism motor gear ratio
    public static final double kFlywheelGearRatio = 1.5;

    // MODE == REAL / REPLAY
    // Feedforward constants
    public static final double kStaticGainReal = 0.1;
    public static final double kVelocityGainReal = 0.05;
    // Feedback (PID) constants
    public static final PIDConstants pidReal = new PIDConstants(1.0, 0.0, 0.0);

    // MODE == SIM
    // Feedforward constants
    public static final double kStaticGainSim = 0.0;
    public static final double kVelocityGainSim = 0.03;
    // Feedback (PID) constants
    public static final PIDConstants pidSim = new PIDConstants(1.0, 0.0, 0.0);
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

  /** Operator Constants *************************************************** */
  public static class OperatorConstants {

    // Joystick Functions
    // Set to TRUE for Drive = Left Stick, Turn = Right Stick; else FALSE
    public static final boolean kDriveLeftTurnRight = true;

    // Joystick Deadbands
    public static final double kDeadband = 0.1;
    public static final double kTurnConstant = 6;

    // Joystick slew rate limiters to smooth erratic joystick motions, measured in units per second
    public static final double kJoystickSlewLimit = 0.5;

    // Override and Console Toggle Switches
    // Assumes this controller: https://www.amazon.com/gp/product/B00UUROWWK
    // Example from:
    // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2024-build-thread/442736/72
    public static final int DRIVER_SWITCH_0 = 1;
    public static final int DRIVER_SWITCH_1 = 2;
    public static final int DRIVER_SWITCH_2 = 3;

    public static final int OPERATOR_SWITCH_0 = 8;
    public static final int OPERATOR_SWITCH_1 = 9;
    public static final int OPERATOR_SWITCH_2 = 10;
    public static final int OPERATOR_SWITCH_3 = 11;
    public static final int OPERATOR_SWITCH_4 = 12;

    public static final int[] MULTI_TOGGLE = {4, 5};
  }

  /** Autonomous Action Constants ****************************************** */
  public static final class AutoConstants {

    // Drive and Turn PID constants used for PathPlanner
    public static final PIDConstants kPPdrivePID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kPPsteerPID = new PIDConstants(5.0, 0.0, 0.0);

    // PathPlanner Config constants
    public static final RobotConfig kPathPlannerConfig =
        new RobotConfig(
            PhysicalConstants.kRobotMassKg,
            PhysicalConstants.kRobotMOI,
            new ModuleConfig(
                SwerveConstants.kWheelRadiusMeters,
                DrivebaseConstants.kMaxLinearSpeed,
                PhysicalConstants.kWheelCOF,
                DCMotor.getKrakenX60Foc(1).withReduction(SwerveConstants.kDriveGearRatio),
                SwerveConstants.kDriveSlipCurrent,
                1),
            Drive.getModuleTranslations());

    // Alternatively, we can build this from the PathPlanner GUI:
    // public static final RobotConfig kPathPlannerConfig = RobotConfig.fromGUISettings();

    // Drive and Turn PID constants used for Chorep
    public static final PIDConstants kChoreoDrivePID = new PIDConstants(10.0, 0.0, 0.0);
    public static final PIDConstants kChoreoSteerPID = new PIDConstants(7.5, 0.0, 0.0);
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

    // Basic filtering thresholds
    public static final double maxAmbiguity = 0.3;
    public static final double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static final double linearStdDevBaseline = 0.02; // Meters
    public static final double angularStdDevBaseline = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static final double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  /** Vision Camera Posses ************************************************* */
  public static class Cameras {
    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "Arducam_OV9281_USB_Camera";
    public static String camera1Name = "camera_1";
    // ... And more, if needed

    // Robot to camera transforms
    // (ONLY USED FOR PHOTONVISION -- Limelight: configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };
  }

  /** List of Device CAN and Power Distribution Circuit IDs **************** */
  public static class CANandPowerPorts {

    /* DRIVETRAIN CAN DEVICE IDS */
    // Input the correct Power Distribution Module port for each motor!!!!
    // NOTE: The CAN ID and bus are set in the Swerve Generator (Phoenix Tuner or YAGSL)

    // Front Left
    public static final RobotDeviceId FL_DRIVE =
        new RobotDeviceId(SwerveConstants.kFLDriveMotorId, SwerveConstants.kFLDriveCanbus, 18);
    public static final RobotDeviceId FL_ROTATION =
        new RobotDeviceId(SwerveConstants.kFLSteerMotorId, SwerveConstants.kFLSteerCanbus, 19);
    public static final RobotDeviceId FL_CANCODER =
        new RobotDeviceId(SwerveConstants.kFLEncoderId, SwerveConstants.kFLEncoderCanbus, null);
    // Front Right
    public static final RobotDeviceId FR_DRIVE =
        new RobotDeviceId(SwerveConstants.kFRDriveMotorId, SwerveConstants.kFRDriveCanbus, 17);
    public static final RobotDeviceId FR_ROTATION =
        new RobotDeviceId(SwerveConstants.kFRSteerMotorId, SwerveConstants.kFRSteerCanbus, 16);
    public static final RobotDeviceId FR_CANCODER =
        new RobotDeviceId(SwerveConstants.kFREncoderId, SwerveConstants.kFREncoderCanbus, null);
    // Back Left
    public static final RobotDeviceId BL_DRIVE =
        new RobotDeviceId(SwerveConstants.kBLDriveMotorId, SwerveConstants.kBLDriveCanbus, 1);
    public static final RobotDeviceId BL_ROTATION =
        new RobotDeviceId(SwerveConstants.kBLSteerMotorId, SwerveConstants.kBLSteerCanbus, 0);
    public static final RobotDeviceId BL_CANCODER =
        new RobotDeviceId(SwerveConstants.kBLEncoderId, SwerveConstants.kBLEncoderCanbus, null);
    // Back Right
    public static final RobotDeviceId BR_DRIVE =
        new RobotDeviceId(SwerveConstants.kBRDriveMotorId, SwerveConstants.kBRSteerCanbus, 2);
    public static final RobotDeviceId BR_ROTATION =
        new RobotDeviceId(SwerveConstants.kBRSteerMotorId, SwerveConstants.kBRSteerCanbus, 3);
    public static final RobotDeviceId BR_CANCODER =
        new RobotDeviceId(SwerveConstants.kBREncoderId, SwerveConstants.kBREncoderCanbus, null);
    // Pigeon
    public static final RobotDeviceId PIGEON =
        new RobotDeviceId(SwerveConstants.kPigeonId, SwerveConstants.kCANbusName, null);

    /* SUBSYSTEM CAN DEVICE IDS */
    // This is where mechanism subsystem devices are defined (Including ID, bus, and power port)
    // Example:
    public static final RobotDeviceId FLYWHEEL_LEADER = new RobotDeviceId(3, "", 8);
    public static final RobotDeviceId FLYWHEEL_FOLLOWER = new RobotDeviceId(4, "", 9);

    /* BEAM BREAK and/or LIMIT SWITCH DIO CHANNELS */
    // This is where digital I/O feedback devices are defined
    // Example:
    // public static final int ELEVATOR_BOTTOM_LIMIT = 3;

    /* LINEAR SERVO PWM CHANNELS */
    // This is where PWM-controlled devices (actuators, servos, pneumatics, etc.)
    // are defined
    // Example:
    // public static final int INTAKE_SERVO = 0;
  }

  /** AprilTag Field Layout ************************************************ */
  /* SEASON SPECIFIC! -- This section is for 2024 (Crescendo) */
  // NOTE: This section will be updated to 2025 "Reefscape" following kickoff
  public static class AprilTagConstants {

    public static final double aprilTagWidth = Units.inchesToMeters(6.50);
    public static final String aprilTagFamily = "36h11";
    public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    @Getter
    public enum AprilTagLayoutType {
      OFFICIAL("2025-official");

      // SPEAKERS_ONLY("2024-speakers"),
      // AMPS_ONLY("2024-amps"),
      // WPI("2024-wpi");

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

  /** Get the current CTRE/Phoenix Pro License state */
  public static CTREPro getPhoenixPro() {
    return phoenixPro;
  }

  /** Get the current AprilTag layout type. */
  public static AprilTagLayoutType getAprilTagLayoutType() {
    return AprilTagConstants.defaultAprilTagType;
  }
}
