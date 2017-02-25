package org.usfirst.frc.team4192.utilities;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Created by Al on 2/23/2017.
 */
public class JaggernautJoystick {
  boolean[] buttons;
  boolean[] lastButtons;
  Joystick joystick;
  
  public JaggernautJoystick(Joystick stick) {
    joystick = stick;
    updateValues();
  }
  
  private void updateValues() {
    if (buttons.length > 0) {
      System.arraycopy(buttons, 0, lastButtons, 0, buttons.length);
    }
    for (int i = 0; i < joystick.getButtonCount(); i++) {
      buttons[i] = joystick.getRawButton(i);
    }
  }
  
  private double createDeadzone(double axisValue) {
    return Math.abs(axisValue) < 0.1 ? 0 : axisValue;
  }
  
  //
  // Run update() once per loop.
  //
  public void update() {
    updateValues();
  }
  
  public boolean get(int index) {
    return joystick.getRawButton(index);
  }
  
  public double getXaxis() {
    return createDeadzone(joystick.getX());
  }
  
  public double getyYaxis() {
    return createDeadzone(joystick.getY());
  }
}
