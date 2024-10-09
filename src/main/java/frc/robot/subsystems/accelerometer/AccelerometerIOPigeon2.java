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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

public class AccelerometerIOPigeon2 implements AccelerometerIO {

  public final String accelName = "Pigeon2";
  private final Pigeon2 pigeon = new Pigeon2(20);
  private final StatusSignal<Double> accelX = pigeon.getAccelerationX();
  private final StatusSignal<Double> accelY = pigeon.getAccelerationY();
  private final StatusSignal<Double> accelZ = pigeon.getAccelerationZ();

  public AccelerometerIOPigeon2() {
    accelX.setUpdateFrequency(100.0);
    accelY.setUpdateFrequency(100.0);
    accelZ.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
  }

  public void updateInputs(AccelerometerIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(accelX, accelY, accelZ).equals(StatusCode.OK);
    inputs.accelXVal = accelX.getValueAsDouble();
    inputs.accelYVal = accelY.getValueAsDouble();
    inputs.accelZVal = accelZ.getValueAsDouble();
  }
}
