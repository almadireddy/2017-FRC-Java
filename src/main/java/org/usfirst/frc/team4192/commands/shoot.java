package org.usfirst.frc.team4192.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4192.Robot;

/**
 * Created by Al on 2/14/2017.
 */
public class shoot extends Command {
  private int time;
  private Timer timer;
  
  public shoot(int timeInSeconds) {
    time = timeInSeconds;
  }
  
  public void initialize() {
    Robot.updateFlywheelConstants();
    Robot.updateFlywheelTargetRPM();
    
    Robot.flywheel.enable();
    Robot.intake.enable();
    Robot.agitator.enable();
  
    timer.start();
  }
  
  public void execute() {
    Robot.intake.set(1);
    Robot.agitator.set(1);
  }
  
  public void end() {
    Robot.flywheel.set(0);
    Robot.intake.set(0);
    Robot.agitator.set(0);
  }
  
  public void interrupted() {
    end();
  }
  
  @Override
  protected boolean isFinished() {
    return (timer.get() >= time);
  }
}