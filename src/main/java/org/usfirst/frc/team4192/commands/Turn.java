package org.usfirst.frc.team4192.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4192.Robot;

/**
 * Created by Al on 2/9/2017.
 */
public class Turn extends Command {
  private double gyroTarget;
  
  public Turn(double degrees) {
    gyroTarget = degrees;
  }
  
  public void initialize() {
    Robot.jankoDrive.prepareForTeleop();
    Robot.turnController.enable();
  }
  
  public void execute() {
    Robot.turnController.setSetpoint(gyroTarget);
  }
  
  public void end() {
    Robot.turnController.disable();
    Robot.frontLeft.set(0);
    Robot.frontRight.set(0);
  }
  
  public void interrupted() {
    end();
  }
  
  @Override
  public boolean isFinished() {
    return Robot.turnController.onTarget();
  }
}
