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

package frc.robot.util;

import static frc.robot.Constants.PowerDistributionConstants.*;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.util.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class PowerMonitoring extends VirtualSubsystem {

  private PowerDistribution powerDistributionModule =
      new PowerDistribution(Ports.POWER_CAN_DEVICE_ID.getDeviceNumber(), kPowerModule);
  private int NUM_PDH_CHANNELS = powerDistributionModule.getNumChannels();
  private double[] channelCurrents = new double[NUM_PDH_CHANNELS];

  /** Periodic Method */
  public void periodic() {

    // Check the total robot current and individual port currents against Constants
    double totalCurrent = powerDistributionModule.getTotalCurrent();
    if (totalCurrent > kTotalMaxCurrent) {
      new Alert("Total current draw exceeds limit!", AlertType.WARNING).set(true);
    }
    for (int i = 0; i < NUM_PDH_CHANNELS; i++) {
      channelCurrents[i] = powerDistributionModule.getCurrent(i);
      if (channelCurrents[i] > kMotorPortMaxCurrent) {
        new Alert("Port " + i + " current draw exceeds limit!", AlertType.WARNING).set(true);
      }
    }

    // Compute DRIVE and STEER summed current
    double driveCurrent = 0.0;
    double steerCurrent = 0.0;
    for (int port : kDrivePowerPorts) {
      driveCurrent += channelCurrents[port];
    }
    for (int port : kSteerPowerPorts) {
      steerCurrent += channelCurrents[port];
    }
    // Add current monitoring by subsystem here
    // Example:
    // double elevatorCurrent = 0.0;
    // for (int port : PowerDistributionConstants.kElevatorPowerPorts) {
    //   elevatorCurrent += channelCurrents[port];
    // }

    // Log values to AdvantageKit and to SmartDashboard
    Logger.recordOutput("PowerMonitor/TotalCurrent", totalCurrent);
    Logger.recordOutput("PowerMonitor/DriveCurrent", driveCurrent);
    Logger.recordOutput("PowerMonitor/SteerCurrent", steerCurrent);
    SmartDashboard.putNumber("TotalCurrent", totalCurrent);
    SmartDashboard.putNumber("DriveCurrent", driveCurrent);
    SmartDashboard.putNumber("SteerCurrent", steerCurrent);
    // Add logging for subsystems here
    // Example:
    // Logger.recordOutput("PowerMonitor/ElevatorCurrent", elevatorCurrent);
    // SmartDashboard.putNumber("ElevatorCurrent", elevatorCurrent);

    // Do something about setting priorities if drawing too much current

  }
}
