package org.usfirst.frc.team4192.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4192.Robot;

/**
 * Created by Al on 2/9/2017.
 */
public class Turn extends Command {
  private double gyroTarget;
  
  public Turn(double degrees) {
    gyroTarget = degrees;
  }
  
  public Turn(double degrees, double p, double i, double d) {
    super(5);
    gyroTarget = degrees;
    Robot.jankoDrive.setPID(p, i, d);
  }
  
  public void initialize() {
    Robot.jankoDrive.prepareForAuton();
    Robot.zeroSensors();
  }
  
  public void execute() {
    Robot.jankoDrive.setSetpoint(gyroTarget);
    SmartDashboard.putString("Left turnOut", ""+Robot.jankoDrive.getLeft());
  }
  
  public void end() {
    Robot.frontLeft.set(0);
    Robot.frontRight.set(0);
    Robot.jankoDrive.disable();
  }
  
  public void interrupted() {
    end();
  }
  
  @Override
  public boolean isFinished() {
    return Robot.jankoDrive.isOnTarget();
  }
}
