package org.usfirst.frc.team4192.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4192.Robot;

/**
 * Created by Al on 2/14/2017.
 */
public class Shoot extends Command {
  private int time;
  private double targetRPM;
  private Timer timer;
  
  public Shoot(int timeInSeconds, double targetRPM) {
    time = timeInSeconds;
    this.targetRPM = targetRPM;
  }
  
  public void initialize() {
    Robot.flywheel.enable();
    timer.start();
  }
  
  public void execute() {
    Robot.flywheel.set(targetRPM);
    Robot.intake.set(1);
    Robot.agitator.set(1);
  }
  
  public void end() {
    Robot.flywheel.disable();
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