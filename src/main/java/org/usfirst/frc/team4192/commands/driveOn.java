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
    driveTarget = inches / (6 * Math.PI); // driveTarget needs to be in wheel revolutions before being set as target. We are using 6 inch wheels.
    SmartDashboard.putNumber("drive target inches", inches);
    SmartDashboard.putNumber("drive target", driveTarget);
  }
  
  public void initialize() {
    Robot.jankoDrive.prepareForAuton();
  }
  
  public void execute() {
    Robot.jankoDrive.setSetpoint(driveTarget);
    SmartDashboard.putBoolean("Drive Forward On", true);
    Robot.jankoDrive.updatePID();
  }
  
  public void end() {
    Robot.jankoDrive.disable();
    SmartDashboard.putBoolean("Drive Forward On", false);
  }
  
  public void interrupted() {
    end();
  }
  
  @Override
  protected boolean isFinished() {
    return false;
  }
}
