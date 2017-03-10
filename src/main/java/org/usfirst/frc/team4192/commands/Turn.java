package org.usfirst.frc.team4192.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4192.Robot;

/**
 * Created by Al on 2/9/2017.
 */
public class Turn extends Command {
  double gyroTarget;
  
  public Turn(double degrees) {
    gyroTarget = degrees;
    SmartDashboard.putNumber("targetHeading", gyroTarget);
  }
  
  public Turn(double degrees, double p, double i, double d) {
    super(5);
    gyroTarget = degrees;
    Robot.jankoDrive.setPID(p, i, d);
    Robot.turnController.reset();
  }
  
  public void initialize() {
    Robot.jankoDrive.prepareForTeleop();
    Robot.zeroSensors();
    Robot.turnController.enable();
  }
  
  public void execute() {
    Robot.turnController.setSetpoint(gyroTarget);
    SmartDashboard.putString("Left turnOut", ""+Robot.jankoDrive.getLeft());
  }
  
  public void end() {
    Robot.leftMaster.set(0);
    Robot.rightMaster.set(0);
    Robot.turnController.disable();
  }
  
  public void interrupted() {
    end();
  }
  
  @Override
  public boolean isFinished() {
    return Robot.gyroOnTarget();
  }
}
