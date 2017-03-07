package org.usfirst.frc.team4192.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4192.Robot;

/**
 * Created by Al on 2/8/2017.
 */
public class driveOn extends Command {
  double driveTarget;
  
  public driveOn(double inches) {
    driveTarget = inches / (6 * Math.PI) * 4096; // driveTarget needs to be in wheel revolutions before being set as target. We are using 6 inch wheels.
  }
  
  public void initialize() {
    Robot.jankoDrive.prepareForSingleEncoderDrive();
    Robot.jankoDrive.zeroSensors();
    Robot.jankoDrive.enable();
  }
  
  public void execute() {
    Robot.jankoDrive.setSetpoint(driveTarget);
    SmartDashboard.putNumber("LeftTarget", driveTarget);
  }
  
  public void end() {
    Robot.jankoDrive.disable();
  }
  
  public void interrupted() {
    end();
  }
  
  @Override
  protected boolean isFinished() {
    return Robot.jankoDrive.isOnTarget();
  }
}
