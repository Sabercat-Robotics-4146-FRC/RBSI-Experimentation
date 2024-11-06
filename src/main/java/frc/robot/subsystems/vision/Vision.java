// Copyright (c) 2024 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024 FRC 2486
// http://github.com/Coconuts2486-FRC
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

package frc.robot.subsystems.vision;

import static frc.robot.Constants.AprilTagConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer.Cameras;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Vision subsystem for AprilTag vision (built upon a virtual subsystem) */
@ExtensionMethod({GeomUtil.class})
public class Vision extends VirtualSubsystem {

  private static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", -(1.0 / 50.0));
  private static final double demoTagPosePersistenceSecs = 0.5;

  // Tag sources and layout
  private final Supplier<AprilTagLayoutType> aprilTagTypeSupplier;
  private final VisionIO[] io;
  private final VisionIOInputs[] inputs;

  // Bookkeeping for last frame exposure and last tag detection
  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  // Something to do with demo?
  private Pose3d demoTagPose = null;
  private double lastDemoTagPoseTimestamp = 0.0;
  public static Pose2d robotPose = null;
  public static Pose3d speakerPose = null;
  private static Translation3d speakerTranslation = null;
  private static Rotation3d robotRotation = null;
  private static Pose3d thisSpeakerPose = null;
  private static Pose3d thisRobotPose = null;

  // Class method definition, including inputs
  public Vision(Supplier<AprilTagLayoutType> aprilTagTypeSupplier, VisionIO... io) {
    this.aprilTagTypeSupplier = aprilTagTypeSupplier;
    this.io = io;
    inputs = new VisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputs();
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      // NOTE: This line causes the RIO to overrun RAM
      // Logger.processInputs("AprilTagVision/Inst" + i, inputs[i]);
    }

    // Loop over cameras
    List<Pose2d> allSpeakerPoses = new ArrayList<>();
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allSpeakerPoses3d = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      var timestamp = inputs[instanceIndex].timestamp;
      var latency = inputs[instanceIndex].latency;
      var targets = inputs[instanceIndex].targets;

      // Exit if no targets
      if (targets == null) {
        continue;
      }

      // Loop over found targets
      for (PhotonTrackedTarget target : targets) {

        // Get the speaker of interest for this alliance
        if ((target.getFiducialId() == 4 && DriverStation.getAlliance().get() == Alliance.Red)
            || (target.getFiducialId() == 7
                && DriverStation.getAlliance().get() == Alliance.Blue)) {

          // Camera pose is "WHERE IS THE CAMERA AS SEEN FROM TAG"; invert to get tag from camera
          Pose3d cameraPose = GeomUtil.toPose3d(target.getBestCameraToTarget().inverse());
          thisRobotPose =
              cameraPose.transformBy(Cameras.cameraPoses[instanceIndex].toTransform3d().inverse());

          // Get the speaker pose from the point of view of the robot
          robotRotation =
              thisRobotPose.getRotation(); // This is rotation of bot w.r.t. tag coordinate system
          speakerTranslation =
              thisRobotPose.getTranslation().unaryMinus().rotateBy(robotRotation.unaryMinus());
          thisSpeakerPose = new Pose3d(speakerTranslation, new Rotation3d());

          allSpeakerPoses3d.add(thisSpeakerPose);
          allSpeakerPoses.add(thisSpeakerPose.toPose2d());
          allRobotPoses.add(thisRobotPose.toPose2d());

          // Log the relevant information to AdvantageKit
          Logger.recordOutput("PhotonVision/Speaker_" + inputs[instanceIndex].camname, speakerPose);
          Logger.recordOutput(
              "PhotonVision/Robot2d_" + inputs[instanceIndex].camname,
              robotRotation.toRotation2d());
        }
      }
    }
    // Set output value based on number of tags seen
    switch ((int) allSpeakerPoses3d.size()) {
      case 0:
        // If no speaker tags, return a null Pose3d
        speakerPose = null;
        robotPose = null;
        break;
      case 1:
        // One tag seen, return it
        speakerPose = allSpeakerPoses3d.get(0);
        robotPose = allRobotPoses.get(0);
        break;
      default:
        // Otherwise, compute the average speakerPose
        Translation3d strans0 = allSpeakerPoses3d.get(0).getTranslation();
        Translation3d strans1 = allSpeakerPoses3d.get(1).getTranslation();
        speakerPose =
            new Pose3d(
                (strans0.getX() + strans1.getX()) / 2.,
                (strans0.getY() + strans1.getY()) / 2.,
                (strans0.getZ() + strans1.getZ()) / 2.,
                new Rotation3d());
        // And then compute the average robotPose
        Translation2d rtrans0 = allRobotPoses.get(0).getTranslation();
        Translation2d rtrans1 = allRobotPoses.get(1).getTranslation();
        Rotation2d rrot0 = allRobotPoses.get(0).getRotation();
        Rotation2d rrot1 = allRobotPoses.get(1).getRotation();
        robotPose =
            new Pose2d(
                (rtrans0.getX() + rtrans1.getX()) / 2.0,
                (rtrans0.getY() + rtrans1.getY()) / 2.0,
                rrot0.plus(rrot1).div(2.0));
    }
  }
}
