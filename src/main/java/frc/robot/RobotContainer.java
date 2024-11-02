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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.AprilTagConstants.AprilTagLayoutType;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.accelerometer.Accelerometer;
import frc.robot.subsystems.flywheel_example.Flywheel;
import frc.robot.subsystems.flywheel_example.FlywheelIO;
import frc.robot.subsystems.flywheel_example.FlywheelIOSim;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveTelemetry;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.util.CanDeviceId;
import frc.robot.util.OverrideSwitches;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  // Define the Driver and, optionally, the Operator/Co-Driver Controllers
  // Replace with ``CommandPS4Controller`` or ``CommandJoystick`` if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);
  final OverrideSwitches overrides = new OverrideSwitches(2);

  // Declare the robot subsystems here
  private final SwerveSubsystem m_drivebase;
  private final Flywheel m_flywheel;
  private final Vision m_vision;
  private final Accelerometer m_accel;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Swerve Drive Telemetry
  private final SwerveTelemetry logger = new SwerveTelemetry(DrivebaseConstants.kMaxLinearSpeed);

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
        m_drivebase = TunerConstants.DriveTrain;
        m_flywheel = new Flywheel(new FlywheelIOSim()); // new Flywheel(new FlywheelIOTalonFX());
        m_vision =
            new Vision(
                this::getAprilTagLayoutType,
                new VisionIOPhoton(this::getAprilTagLayoutType, "Photon_CAMNAME"));
        m_accel = new Accelerometer(m_drivebase.getPigeon2());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_drivebase = TunerConstants.DriveTrain;
        m_flywheel = new Flywheel(new FlywheelIOSim());
        m_vision = new Vision(this::getAprilTagLayoutType);
        m_accel = new Accelerometer(m_drivebase.getPigeon2());
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drivebase = TunerConstants.DriveTrain;
        m_flywheel = new Flywheel(new FlywheelIO() {});
        m_vision = new Vision(this::getAprilTagLayoutType, new VisionIO() {}, new VisionIO() {});
        m_accel = new Accelerometer(m_drivebase.getPigeon2());
        break;
    }

    // Configure the trigger bindings
    configureBindings();
    // Define Auto commands
    defineAutoCommands();
    // Set up the SmartDashboard Auto Chooser
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up the logger
    m_drivebase.registerTelemetry(logger::telemeterize);
  }

  /** Use this method to define your Autonomous commands for use with PathPlanner / Choreo */
  private void defineAutoCommands() {

    NamedCommands.registerCommand("Zero", Commands.runOnce(() -> m_drivebase.tareEverything()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Manually Re-Zero the Gyro & Odometry
    driverXbox.y().onTrue(Commands.runOnce(() -> m_drivebase.tareEverything()));

    // Example Swerve Drive button bindings
    // A -> BRAKE
    driverXbox.a().whileTrue(m_drivebase.applyRequest(() -> m_drivebase.brake));
    // B -> POINT WHEELS AT DIRECTION WITHOUT MOVING
    driverXbox
        .b()
        .whileTrue(
            m_drivebase.applyRequest(
                () ->
                    m_drivebase.point.withModuleDirection(
                        new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX()))));
    // LEFT BUMPER -> reset the field-centric heading
    driverXbox.leftBumper().onTrue(m_drivebase.runOnce(() -> m_drivebase.seedFieldRelative()));
    // POV UP -> DRIVE FORWARD IN ROBOT-CENTRIC MODE
    driverXbox
        .pov(0)
        .whileTrue(
            m_drivebase.applyRequest(
                () -> m_drivebase.forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    // POV DOWN -> DRIVE BACKWARD IN ROBOT-CENTRIC MODE
    driverXbox
        .pov(180)
        .whileTrue(
            m_drivebase.applyRequest(
                () -> m_drivebase.forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

    // SET STANDARD DRIVING AS DEFAULT COMMAND FOR THE DRIVEBASE
    // NOTE: Left joystick controls lateral translation, right joystick (X) controls rotation
    m_drivebase.setDefaultCommand(
        m_drivebase
            .applyRequest(
                () ->
                    m_drivebase
                        .drive
                        .withVelocityX(-driverXbox.getLeftY() * DrivebaseConstants.kMaxLinearSpeed)
                        .withVelocityY(-driverXbox.getLeftX() * DrivebaseConstants.kMaxLinearSpeed)
                        .withRotationalRate(
                            -driverXbox.getRightX() * DrivebaseConstants.kMaxAngularSpeed))
            .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An example command will be run in autonomous
    return m_drivebase.getAutoPath("Tests");
    // Use the ``autoChooser`` to define your auto path from the SmartDashboard
    // return autoChooser.get();
  }

  public void setDriveMode() {
    configureBindings();
  }

  /** Set the motor neutral mode to BRAKE / COAST for T/F */
  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  /** List of Channel and CAN IDs ****************************************** */
  public static class Ports {

    /* DRIVETRAIN CAN DEVICE IDS */
    // This is the default setup for the Az-RBSI swerve base
    // Swerve Modules go:
    // FL,FR,BL,BR
    //
    // 0 1
    // 2 3
    public static final CanDeviceId FL_DRIVE = new CanDeviceId(1, "canivore");
    public static final CanDeviceId FL_ROTATION = new CanDeviceId(2, "canivore");
    public static final CanDeviceId FL_CANCODER = new CanDeviceId(3, "canivore");

    public static final CanDeviceId FR_DRIVE = new CanDeviceId(4, "canivore");
    public static final CanDeviceId FR_ROTATION = new CanDeviceId(5, "canivore");
    public static final CanDeviceId FR_CANCODER = new CanDeviceId(6, "canivore");

    public static final CanDeviceId BL_DRIVE = new CanDeviceId(7, "canivore");
    public static final CanDeviceId BL_ROTATION = new CanDeviceId(8, "canivore");
    public static final CanDeviceId BL_CANCODER = new CanDeviceId(9, "canivore");

    public static final CanDeviceId BR_DRIVE = new CanDeviceId(10, "canivore");
    public static final CanDeviceId BR_ROTATION = new CanDeviceId(11, "canivore");
    public static final CanDeviceId BR_CANCODER = new CanDeviceId(12, "canivore");

    public static final CanDeviceId PIGEON = new CanDeviceId(13, "canivore");

    /* POWER DISTRIBUTION CAN ID */
    public static final CanDeviceId POWER_CAN_DEVICE_ID = new CanDeviceId(1, "");

    /* SUBSYSTEM CAN DEVICE IDS */
    // This is where mechanism subsystem devices are defined
    // Example:
    public static final CanDeviceId FLYWHEEL_LEADER = new CanDeviceId(15, "");
    public static final CanDeviceId FLYWHEEL_FOLLOWER = new CanDeviceId(16, "");

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

  /** Vision Constants and Camera Posses *********************************** */
  public static class VisionConstants {
    public static final double ambiguityThreshold = 0.4;
    public static final double targetLogTimeSecs = 0.1;
    public static final double fieldBorderMargin = 0.5;
    public static final double zMargin = 0.75;
    public static final double xyStdDevCoefficient = 0.005;
    public static final double thetaStdDevCoefficient = 0.01;

    public static final Pose3d[] cameraPoses =
        switch (Constants.getRobot()) {
          case COMPBOT ->
              new Pose3d[] {
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
