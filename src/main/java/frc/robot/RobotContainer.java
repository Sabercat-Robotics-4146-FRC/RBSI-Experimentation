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
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.robot.commands.ChoreoAutoController;
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
import frc.robot.util.RBSIEnum;
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

  // Declare the robot subsystems here
  // These are the "Active Subsystems" that the robot controlls
  private final Drive m_drivebase;
  private final Flywheel m_flywheel;
  // These are "Virtual Subsystems" that report information but have no motors
  private final Accelerometer m_accel;
  private final Vision m_vision;
  private final PowerMonitoring m_power;

  // Dashboard inputs
  // AutoChoosers for both supported path planning types
  private final LoggedDashboardChooser<Command> autoChooserPathPlanner;
  private final AutoChooser autoChooserChoreo;
  private final AutoFactory autoFactoryChoreo;
  private final ChoreoAutoController choreoController;
  // Input estimated battery capacity (if full, use printed value)
  private final LoggedTunableNumber batteryCapacity =
      new LoggedTunableNumber("Battery Amp-Hours", 18.0);
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
        m_flywheel = new Flywheel(new FlywheelIOSim() {});
        m_vision =
            new Vision(
                m_drivebase::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_drivebase::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, m_drivebase::getPose));
        m_accel = new Accelerometer(m_drivebase.getGyro());
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

    // In addition to the initial battery capacity from the Dashbaord, ``PowerMonitoring`` takes all
    // the non-drivebase subsystems for which you wish to have power monitoring; DO NOT include
    // ``m_drivebase``, as that is automatically monitored.
    m_power = new PowerMonitoring(batteryCapacity, m_flywheel);

    // Set up the SmartDashboard Auto Chooser based on auto type
    switch (Constants.getAutoType()) {
      case PATHPLANNER:
        autoChooserPathPlanner =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        // Set the others to null
        autoChooserChoreo = null;
        autoFactoryChoreo = null;
        choreoController = null;
        break;
      case CHOREO:
        choreoController = new ChoreoAutoController(m_drivebase);
        autoFactoryChoreo =
            Choreo.createAutoFactory(
                m_drivebase::getPose, // A function that returns the current robot pose
                choreoController, // The controller for the drive subsystem
                this::isRedAlliance, // A function that returns true if the robot is on the red
                // alliance
                m_drivebase,
                new AutoBindings() // An empty `AutoBindings` object, you can learn more below
                );
        autoChooserChoreo = new AutoChooser(autoFactoryChoreo, "");
        autoChooserChoreo.addAutoRoutine("twoPieceAuto", this::twoPieceAuto);
        // Set the others to null
        autoChooserPathPlanner = null;
        break;
      default:
        // Then, throw the error
        throw new RuntimeException(
            "Incorrect AUTO type selected in Constants: " + Constants.getAutoType());
    }

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

  /**
   * Set up the SysID routines from AdvantageKit
   *
   * <p>NOTE: These are currently only accessible with Constants.AutoType.PATHPLANNER
   */
  private void definesysIdRoutines() {
    if (Constants.getAutoType() == RBSIEnum.AutoType.PATHPLANNER) {
      // Drivebase characterization
      autoChooserPathPlanner.addOption(
          "Drive Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive Simple FF Characterization",
          DriveCommands.feedforwardCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Forward)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Reverse)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Forward)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Reverse)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));

      // Example Flywheel SysId Characterization
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Quasistatic Forward)",
          m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Quasistatic Reverse)",
          m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Dynamic Forward)",
          m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Flywheel SysId (Dynamic Reverse)",
          m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
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
  public Command getAutonomousCommandPathPlanner() {
    // Use the ``autoChooser`` to define your auto path from the SmartDashboard
    return autoChooserPathPlanner.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public AutoRoutine getAutonomousCommandChoreo() {
    return autoChooserChoreo.getSelectedAutoRoutine();
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

  /**
   * Example Choreo auto command
   *
   * <p>NOTE: This would normally be in a spearate file.
   */
  private AutoRoutine twoPieceAuto(AutoFactory factory) {
    final AutoRoutine routine = factory.newRoutine("twoPieceAuto");

    final AutoTrajectory trajectory = routine.trajectory("twoPieceAuto");

    routine
        .running()
        .onTrue(
            m_drivebase
                .resetOdometry(
                    trajectory
                        .getInitialPose()
                        .orElseGet(
                            () -> {
                              routine.kill();
                              return new Pose2d();
                            }))
                .andThen(trajectory.cmd())
                .withName("twoPieceAuto entry point"));

    // trajectory.atTime("intake").onTrue(intake.extend());
    // trajectory.atTime("shoot").onTrue(shooter.launch());

    return routine;
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Red);
  }
}
