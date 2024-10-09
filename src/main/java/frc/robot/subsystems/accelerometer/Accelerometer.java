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

package frc.robot.subsystems.accelerometer;

import frc.robot.subsystems.accelerometer.AccelerometerIO.AccelerometerIOInputs;
import frc.robot.util.VirtualSubsystem;

/** Accelerometer subsystem (built upon a virtual subsystem) */
public class Accelerometer extends VirtualSubsystem {

  private final AccelerometerIO[] io;
  private final AccelerometerIOInputs[] inputs;

  // Class method definition, including inputs
  public Accelerometer(AccelerometerIO... io) {
    this.io = io;
    inputs = new AccelerometerIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new AccelerometerIOInputs();
    }
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
    }
  }
}
