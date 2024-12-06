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
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.Choreo;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.AprilTagConstants.AprilTagLayoutType;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PowerConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.accelerometer.Accelerometer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel_example.Flywheel;
import frc.robot.subsystems.flywheel_example.FlywheelIO;
import frc.robot.subsystems.flywheel_example.FlywheelIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OverrideSwitches;
import frc.robot.util.PowerMonitoring;
import frc.robot.util.RobotDeviceId;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** This is the location for defining robot hardware, commands, and controller button bindings. */
public class RobotContainer {

  // Define the Driver and, optionally, the Operator/Co-Driver Controllers
  // Replace with ``CommandPS4Controller`` or ``CommandJoystick`` if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  final OverrideSwitches overrides = new OverrideSwitches(2);

  // Autonomous Things
  Field2d m_field = new Field2d();
  Trajectory m_traj;

  // Declare the robot subsystems here
  private final Drive m_drivebase;
  private final Flywheel m_flywheel;
  private final Accelerometer m_accel;
  private final Vision m_vision;
  private final PowerMonitoring m_power;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // EXAMPLE TUNABLE FLYWHEEL SPEED INPUT FROM DASHBOARD
  private final LoggedTunableNumber flywheelSpeedInput =
      new LoggedTunableNumber("Flywheel Speed", 1500.0);

  // Alerts
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getAprilTagLayoutType() {
    return AprilTagConstants.defaultAprilTagType;
  }

  public static AprilTagLayoutType staticGetAprilTagLayoutType() {
    return AprilTagConstants.defaultAprilTagType;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Instantiate Robot Subsystems based on RobotType
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // YAGSL drivebase, get config from deploy directory
        m_drivebase = new Drive();
        m_flywheel = new Flywheel(new FlywheelIOSim()); // new Flywheel(new FlywheelIOTalonFX());
        m_vision =
            switch (Constants.getVisionType()) {
              case PHOTON ->
                  new Vision(
                      m_drivebase::addVisionMeasurement,
                      new VisionIOPhotonVision(camera0Name, robotToCamera0),
                      new VisionIOPhotonVision(camera1Name, robotToCamera1));
              case LIMELIGHT ->
                  new Vision(
                      m_drivebase::addVisionMeasurement,
                      new VisionIOLimelight(camera0Name, m_drivebase::getRotation),
                      new VisionIOLimelight(camera1Name, m_drivebase::getRotation));
              case NONE ->
                  new Vision(
                      m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
              default -> null;
            };
        m_accel = new Accelerometer(m_drivebase.getGyro());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_drivebase = new Drive();
        m_vision =
            new Vision(
                m_drivebase::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_drivebase::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, m_drivebase::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drivebase = new Drive();
        m_flywheel = new Flywheel(new FlywheelIO() {});
        m_vision =
            new Vision(m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        m_accel = new Accelerometer(m_drivebase.getGyro());
        break;
    }

    // ``PowerMonitoring`` takes all the non-drivebase subsystems for which
    //   you wish to have power monitoring; DO NOT include ``m_drivebase``,
    //   as that is automatically monitored.
    m_power = null; // new PowerMonitoring(m_flywheel);

    // Set up the SmartDashboard Auto Chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the trigger bindings
    configureBindings();
    // Define Auto commands
    defineAutoCommands();
    // Define SysIs Routines
    definesysIdRoutines();
  }

  /** Use this method to define your Autonomous commands for use with PathPlanner / Choreo */
  private void defineAutoCommands() {

    NamedCommands.registerCommand("Zero", Commands.runOnce(() -> m_drivebase.zero()));
  }

  /** Set up the SysID routines from AdvantageKit */
  private void definesysIdRoutines() {
    // Drivebase characterization
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(m_drivebase));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drivebase));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Example Flywheel SysId Characterization
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Forward)",
        m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Quasistatic Reverse)",
        m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Forward)",
        m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Flywheel SysId (Dynamic Reverse)",
        m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {

    // SET STANDARD DRIVING AS DEFAULT COMMAND FOR THE DRIVEBASE
    // NOTE: Left joystick controls lateral translation, right joystick (X) controls rotation
    m_drivebase.setDefaultCommand(
        DriveCommands.fieldRelativeDrive(
            m_drivebase,
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX(),
            () -> driverXbox.getRightX()));

    // Example Commands
    // Press B button while driving --> ROBOT-CENTRIC
    driverXbox
        .b()
        .onTrue(
            Commands.runOnce(
                () ->
                    DriveCommands.robotRelativeDrive(
                        m_drivebase,
                        () -> -driverXbox.getLeftY(),
                        () -> -driverXbox.getLeftX(),
                        () -> driverXbox.getRightX()),
                m_drivebase));

    // Press A button -> BRAKE
    driverXbox.a().whileTrue(Commands.runOnce(() -> m_drivebase.setMotorBrake(true), m_drivebase));

    // Press X button --> Stop with wheels in X-Lock position
    driverXbox.x().onTrue(Commands.runOnce(m_drivebase::stopWithX, m_drivebase));

    // Press Y button --> Manually Re-Zero the Gyro
    driverXbox.y().onTrue(Commands.runOnce(() -> m_drivebase.zero()));

    // Press RIGHT BUMPER --> Run the example flywheel
    driverXbox
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> m_flywheel.runVelocity(flywheelSpeedInput.get()),
                m_flywheel::stop,
                m_flywheel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An example command will be run in autonomous
    // return m_drivebase.getAutoPath("Tests");
    // Use the ``autoChooser`` to define your auto path from the SmartDashboard
    return autoChooser.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandChoreo() {

    m_traj = Choreo.getTrajectory("Trajectory");

    m_field.getObject("traj").setPoses(m_traj.getInitialPose(), m_traj.getFinalPose());
    m_field.getObject("trajPoses").setPoses(m_traj.getPoses());

    var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_drivebase.setPose(m_traj.getInitialPose());

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Command swerveCommand =
        Choreo.choreoSwerveCommand(
            // Choreo trajectory from above
            m_traj,
            // A function that returns the current field-relative pose of the robot: your wheel or
            // vision odometry
            m_drivebase::getPose,
            // PIDController for field-relative X translation (input: X error meters, output: m/s).
            new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0),
            // PIDController for field-relative Y translation (input: Y error meters, output: m/s).
            new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0),
            // PID constants to correct for rotation error
            thetaController,
            (ChassisSpeeds speeds) ->
                m_drivebase.runVelocity(
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond,
                        isFlipped
                            ? m_drivebase.getRotation().plus(new Rotation2d(Math.PI))
                            : m_drivebase.getRotation())),
            // Whether or not to mirror the path based on alliance (this assumes the path is created
            // for the blue alliance)
            () -> true,
            // The subsystem(s) to require, typically your drive subsystem only
            m_drivebase);

    return Commands.sequence(
        Commands.runOnce(() -> m_drivebase.setPose(m_traj.getInitialPose())),
        swerveCommand,
        m_drivebase.run(() -> m_drivebase.stop()));
  }

  public void setDriveMode() {
    configureBindings();
  }

  /** Set the motor neutral mode to BRAKE / COAST for T/F */
  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  /** Updates the alerts. */
  public void updateAlerts() {
    // AprilTag layout alert
    boolean aprilTagAlertActive = getAprilTagLayoutType() != AprilTagLayoutType.OFFICIAL;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use (" + getAprilTagLayoutType().toString() + ").");
    }
  }

  /** List of Device CAN and Power Distribution Circuit IDs **************** */
  public static class Ports {

    /* DRIVETRAIN CAN DEVICE IDS */
    // This is the default setup for the Az-RBSI swerve base
    // Swerve Modules go:
    // FL,FR,BL,BR
    //
    // 0 1
    // 2 3
    public static final RobotDeviceId FL_DRIVE = new RobotDeviceId(1, "DriveTrain", 18);
    public static final RobotDeviceId FL_ROTATION = new RobotDeviceId(2, "DriveTrain", 19);
    public static final RobotDeviceId FL_CANCODER = new RobotDeviceId(3, "DriveTrain", null);

    public static final RobotDeviceId FR_DRIVE = new RobotDeviceId(4, "DriveTrain", 17);
    public static final RobotDeviceId FR_ROTATION = new RobotDeviceId(5, "DriveTrain", 16);
    public static final RobotDeviceId FR_CANCODER = new RobotDeviceId(6, "DriveTrain", null);

    public static final RobotDeviceId BL_DRIVE = new RobotDeviceId(7, "DriveTrain", 1);
    public static final RobotDeviceId BL_ROTATION = new RobotDeviceId(8, "DriveTrain", 0);
    public static final RobotDeviceId BL_CANCODER = new RobotDeviceId(9, "DriveTrain", null);

    public static final RobotDeviceId BR_DRIVE = new RobotDeviceId(10, "DriveTrain", 2);
    public static final RobotDeviceId BR_ROTATION = new RobotDeviceId(11, "DriveTrain", 3);
    public static final RobotDeviceId BR_CANCODER = new RobotDeviceId(12, "DriveTrain", null);

    public static final RobotDeviceId PIGEON = new RobotDeviceId(13, "DriveTrain", null);

    /* POWER DISTRIBUTION CAN ID (set by device type in PowerConstants) */
    public static final RobotDeviceId POWER_CAN_DEVICE_ID =
        switch (PowerConstants.kPowerModule) {
          case kRev -> new RobotDeviceId(1, null);
          case kCTRE -> new RobotDeviceId(0, null);
          default -> null;
        };

    /* SUBSYSTEM CAN DEVICE IDS */
    // This is where mechanism subsystem devices are defined
    // Example:
    public static final RobotDeviceId FLYWHEEL_LEADER = new RobotDeviceId(3, 8);
    public static final RobotDeviceId FLYWHEEL_FOLLOWER = new RobotDeviceId(4, 9);

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

  /** Override and Console Toggle Switches ********************************* */
  public static class Overrides {

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

  /** Vision Camera Posses ************************************************* */
  public static class Cameras {

    public static final Pose3d[] cameraPoses =
        switch (Constants.getRobot()) {
          case COMPBOT ->
              new Pose3d[] {
                // Camera #1
                new Pose3d(
                    Units.inchesToMeters(-1.0),
                    Units.inchesToMeters(0),
                    Units.inchesToMeters(23.5),
                    new Rotation3d(0.0, Units.degreesToRadians(-20), 0.0)),
              };
          case DEVBOT -> new Pose3d[] {};
          default -> new Pose3d[] {};
        };
  }
}
