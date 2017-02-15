package org.usfirst.frc.team4192.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4192.Robot;

/**
 * Created by Al on 2/8/2017.
 */
public class driveOn extends Command {
  double driveTarget;
  double targetCounter;
  
  public driveOn(double inches) {   // 4096 units per revolution
    driveTarget = inches * (4096/(6*Math.PI)); // Scale factor for native units to inches: 4096/6pi = ticks/inch
    targetCounter = 0;
  }
  
  public void initialize() {
    Robot.switchDriveMotorsToPositionControl();
  }
  
  public void execute() {
    Robot.frontLeft.set(driveTarget);
    Robot.frontRight.set(driveTarget);
    if (Robot.driveOnTarget())
      targetCounter++;
  }
  
  public void end() {
    Robot.frontLeft.disable();
    Robot.frontRight.disable();
    Robot.switchDriveMotorsToDefaultControl();
  }
  
  public void interrupted() {
    end();
  }
  
  @Override
  protected boolean isFinished() {
    return Robot.driveOnTarget() && targetCounter >= 100;
  }
}
