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

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {

  class VisionIOInputs implements LoggableInputs {
    public String camname = "";
    public double latency = 0.0;
    public double timestamp = 0.0;
    public List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>() {};

    public long fps = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Latency", latency);
      table.put("Timestamp", timestamp);
      table.put("TargetCount", targets.size());
      for (int i = 0; i < targets.size(); i++) {
        table.put("Target/" + i, targets.get(i));
      }
      table.put("Fps", fps);
    }

    @Override
    public void fromLog(LogTable table) {
      latency = table.get("Latency", 0.0);
      timestamp = table.get("Timestamp", 0.0);
      int targetCount = table.get("TargetCount", 0);
      targets = new ArrayList<PhotonTrackedTarget>(targetCount);
      fps = table.get("Fps", 0);
    }
  }

  default void updateInputs(VisionIOInputs inputs) {}
}
