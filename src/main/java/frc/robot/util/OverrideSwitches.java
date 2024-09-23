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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for physical override switches on operator console. */
public class OverrideSwitches {
  private final GenericHID joystick;

  public OverrideSwitches(int port) {
    joystick = new GenericHID(port);
  }

  /** Returns whether the controller is connected. */
  public boolean isConnected() {
    return joystick.isConnected()
        && !DriverStation.getJoystickIsXbox(joystick.getPort())
        && joystick.getName().equals("Generic   USB  Joystick");
  }

  /** Gets the state of a driver-side switch (0-2 from left to right). */
  public boolean getDriverSwitch(int index) {
    if (index < 0 || index > 2) {
      throw new RuntimeException(
          "Invalid driver override index " + Integer.toString(index) + ". Must be 0-2.");
    }
    return joystick.getRawButton(index + 1);
  }

  /** Gets the state of an operator-side switch (0-4 from left to right). */
  public boolean getOperatorSwitch(int index) {
    if (index < 0 || index > 4) {
      throw new RuntimeException(
          "Invalid operator override index " + Integer.toString(index) + ". Must be 0-4.");
    }
    return joystick.getRawButton(index + 8);
  }

  /** Gets the state of the multi-directional switch. */
  public MultiDirectionSwitchState getMultiDirectionSwitch() {
    if (joystick.getRawButton(4)) {
      return MultiDirectionSwitchState.LEFT;
    } else if (joystick.getRawButton(5)) {
      return MultiDirectionSwitchState.RIGHT;
    } else {
      return MultiDirectionSwitchState.NEUTRAL;
    }
  }

  /** Returns a trigger for a driver-side switch (0-2 from left to right). */
  public Trigger driverSwitch(int index) {
    return new Trigger(() -> getDriverSwitch(index));
  }

  /** Returns a trigger for an operator-side switch (0-4 from left to right). */
  public Trigger operatorSwitch(int index) {
    return new Trigger(() -> getOperatorSwitch(index));
  }

  /** Returns a trigger for when the multi-directional switch is pushed to the left. */
  public Trigger multiDirectionSwitchLeft() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.LEFT);
  }

  /** Returns a trigger for when the multi-directional switch is pushed to the right. */
  public Trigger multiDirectionSwitchRight() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.RIGHT);
  }

  /** The state of the multi-directional switch. */
  public static enum MultiDirectionSwitchState {
    LEFT,
    NEUTRAL,
    RIGHT
  }
}
