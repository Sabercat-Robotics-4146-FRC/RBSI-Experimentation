// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
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

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants2024;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double apply(double xCoordinate) {
    if (shouldFlip()) {
      return FieldConstants2024.fieldLength - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(apply(translation.getX()), translation.getY());
    } else {
      return translation;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    } else {
      return pose;
    }
  }

  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip()) {
      return new Translation3d(
          apply(translation3d.getX()), translation3d.getY(), translation3d.getZ());
    } else {
      return translation3d;
    }
  }

  //   /**
  //    * Flips a trajectory state to the correct side of the field based on the current alliance
  // color.
  //    */
  //   public static VehicleState apply(VehicleState state) {
  //     if (shouldFlip()) {
  //       return VehicleState.newBuilder()
  //           .setX(apply(state.getX()))
  //           .setY(state.getY())
  //           .setTheta(apply(new Rotation2d(state.getTheta())).getRadians())
  //           .setVx(-state.getVx())
  //           .setVy(state.getVy())
  //           .setOmega(-state.getOmega())
  //           .addAllModuleForces(
  //               state.getModuleForcesList().stream()
  //                   .map(
  //                       forces ->
  //                           VehicleTrajectoryServiceOuterClass.ModuleForce.newBuilder()
  //                               .setFx(-forces.getFx())
  //                               .setFy(forces.getFy())
  //                               .build())
  //                   .toList())
  //           .build();
  //     } else {
  //       return state;
  //     }
  //   }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
