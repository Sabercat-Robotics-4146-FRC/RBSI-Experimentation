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

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AprilTagLayoutType;
import frc.robot.util.Alert;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhoton implements VisionIO {

  private final Supplier<AprilTagLayoutType> aprilTagTypeSupplier;
  private AprilTagLayoutType lastAprilTagType = null;

  private static final double disconnectedTimeout = 0.5;
  private final Alert disconnectedAlert;
  private final Timer disconnectedTimer = new Timer();
  public final PhotonCamera camera;
  public final String camname;

  public VisionIOPhoton(Supplier<AprilTagLayoutType> aprilTagTypeSupplier, String camname) {
    this.aprilTagTypeSupplier = aprilTagTypeSupplier;
    this.camname = camname;
    this.camera = new PhotonCamera(camname);

    disconnectedAlert = new Alert("No data from \"" + camname + "\"", Alert.AlertType.ERROR);
    disconnectedTimer.start();
  }

  public void updateInputs(VisionIOInputs inputs) {
    // Get observations
    var result = camera.getLatestResult();

    // Log the entire result for each camera to AdvantageKit
    Logger.recordOutput("PhotonVision/" + camname, result); // result.getLatencyMillis());

    // Put the relevant information into the `inputs`
    inputs.camname = camname;
    inputs.latency = result.getLatencyMillis();
    inputs.timestamp = result.getTimestampSeconds();

    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      inputs.targets = targets;
      for (PhotonTrackedTarget target : targets) {
        // Get information from target.
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

        String logTag = "PhotonVision/Tag" + Integer.toString(targetID);
        Logger.recordOutput(logTag + "/PoseAmbiguity", poseAmbiguity);
        Logger.recordOutput(logTag + "/BestCameraToTarget", bestCameraToTarget);
        Logger.recordOutput(logTag + "/AlternateCameraToTarget", alternateCameraToTarget);
      }
    } else {
      inputs.targets = null;
    }
  }
}
