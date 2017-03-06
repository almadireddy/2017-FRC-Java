package org.usfirst.frc.team4192.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4192.Robot;

/**
 * Created by Al on 2/8/2017.
 */
public class driveOn extends Command {
  double driveTarget;
  
  public driveOn(double inches) {
    driveTarget = inches / (6 * Math.PI); // driveTarget needs to be in wheel revolutions before being set as target. We are using 6 inch wheels.
  }
  
  public void initialize() {
    Robot.jankoDrive.prepareForSingleEncoderDrive();
  }
  
  public void execute() {
    Robot.frontLeft.set(driveTarget);
  }
  
  public void end() {
    Robot.frontLeft.disable();
    Robot.frontRight.disable();
  }
  
  public void interrupted() {
    end();
  }
  
  @Override
  protected boolean isFinished() {
    return Robot.jankoDrive.isOnTarget();
  }
}
