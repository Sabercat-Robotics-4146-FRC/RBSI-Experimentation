// Copyright (c) 2024 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2024 FRC 254
// https://github.com/team254
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

/** Class for wrapping CAN devices with a name and functionality */
public class CanDeviceId {
  private final int mDeviceNumber;
  private final String mBus;

  public CanDeviceId(int deviceNumber, String bus) {
    mDeviceNumber = deviceNumber;
    mBus = bus;
  }

  /** Use the default bus name (empty string) */
  public CanDeviceId(int deviceNumber) {
    this(deviceNumber, "");
  }

  /** Get the CAN ID value for a named device */
  public int getDeviceNumber() {
    return mDeviceNumber;
  }

  /** Get the CAN bus name for a named device */
  public String getBus() {
    return mBus;
  }

  /** Check whether two named devices are, in fact, the same */
  public boolean equals(CanDeviceId other) {
    return other.mDeviceNumber == mDeviceNumber && other.mBus == mBus;
  }
}
