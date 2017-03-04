package org.usfirst.frc.team4192.utilities;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Created by Al on 2/23/2017.
 */
public class JaggernautJoystick {
  private boolean[] buttons;
  private boolean[] lastButtons;
  private Joystick joystick;
  
  public JaggernautJoystick(int port) {
    joystick = new Joystick(port);
    buttons = new boolean[joystick.getButtonCount()];
    lastButtons = new boolean[joystick.getButtonCount()];
    updateButtonStates();
  }
  
  public Joystick getJoystick() {
    return joystick;
  }
  
  private void updateButtonStates() {
    if (buttons.length > 0) {
      System.arraycopy(buttons, 0, lastButtons, 0, buttons.length);
    }
    for (int i = 1; i < joystick.getButtonCount(); i++) {
      buttons[i] = joystick.getRawButton(i);
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
  
  public boolean isHeldDown(int button) {
    return buttons[button] && lastButtons[button];
  }
  
  public boolean buttonPressed(int button) {
    return buttons[button] && !lastButtons[button];
  }
  
  public boolean buttonReleased(int button) {
    return !buttons[button] && lastButtons[button];
  }
  
  public void rumble() {
    joystick.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
    joystick.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
  }
}
