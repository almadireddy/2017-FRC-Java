package org.usfirst.frc.team4192.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by Al on 2/23/2017.
 */
public class JaggernautJoystick {
  private boolean[] currentButtons;
  private boolean[] lastButtons;
  private Joystick joystick;
  
  public JaggernautJoystick(int port) {
    joystick = new Joystick(port);
    SmartDashboard.putNumber("Joystick Buttons", joystick.getButtonCount());
    currentButtons = new boolean[11];
    lastButtons = new boolean[11];
    updateButtonStates();
  }
  
  public Joystick getJoystick() {
    return joystick;
  }
  
  private void updateButtonStates() {
    if (currentButtons.length > 0) {
      System.arraycopy(currentButtons, 0, lastButtons, 0, currentButtons.length);
    }
    for (int i = 1; i < joystick.getButtonCount(); i++) {
      currentButtons[i] = joystick.getRawButton(i);
    }
  }
  
  private double createDeadzone(double axisValue) {
    return Math.abs(axisValue) < 0.1 ? 0 : axisValue;
  }
  
  // Run update() in the periodic function.
  public void update() {
    updateButtonStates();
  }
  
  public boolean get(int button) {
    return joystick.getRawButton(button);
  }
  
  public double getXaxis() {
    return createDeadzone(joystick.getRawAxis(4));
  }
  
  public double getYaxis() {
    return createDeadzone(joystick.getY());
  }
  
  public double getLeftTrigger() {
    return joystick.getRawAxis(2);
  }
  
  public double getRightTrigger() {
    return joystick.getRawAxis(3);
  }
  
  public double getDPad(int button) {
    return joystick.getPOV(1);
  }
  
  public double getAxis(int axis) {
    return joystick.getRawAxis(axis);
  }
  
  public boolean isHeldDown(int button) {
    return currentButtons[button] && lastButtons[button];
  }
  
  public boolean buttonPressed(int button) {
    return currentButtons[button] && !lastButtons[button];
  }
  
  public boolean buttonReleased(int button) {
    return !currentButtons[button] && lastButtons[button];
  }
  
  public void rumble() {
    joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
  }
}
