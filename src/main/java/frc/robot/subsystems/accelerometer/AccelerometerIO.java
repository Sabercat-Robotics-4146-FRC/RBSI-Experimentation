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

package frc.robot.subsystems.accelerometer;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AccelerometerIO {
  @AutoLog
  class AccelerometerIOInputs implements LoggableInputs {
    public boolean connected = false;
    public double accelXVal = 0.0;
    public double accelYVal = 0.0;
    public double accelZVal = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("AccelX", accelXVal);
      table.put("AccelY", accelYVal);
      table.put("AccelZ", accelZVal);
    }

    @Override
    public void fromLog(LogTable table) {
      accelXVal = table.get("AccelX", 0.0);
      accelYVal = table.get("AccelY", 0.0);
      accelZVal = table.get("AccelZ", 0.0);
    }
  }

  default void updateInputs(AccelerometerIOInputs inputs) {}
}
