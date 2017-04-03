package org.usfirst.frc.team4192.commands;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4192.JankoConstants;
import org.usfirst.frc.team4192.Robot;

/**
 * Created by darkb on 3/29/2017.
 */
public class ActuatorForward extends Command {
  
  public ActuatorForward() {
    super();
  }
  
  @Override
  protected void initialize() {
    Robot.actuator.changeControlMode(CANTalon.TalonControlMode.Position);
    Robot.actuator.enable();
  }
  
  @Override
  protected void execute() {
    Robot.actuator.setSetpoint(JankoConstants.gearScorePosition);
  }
  
  @Override
  protected boolean isFinished() {
    return false;
  }
  
  @Override
  protected void end() {
    Robot.actuator.disable();
  }
  
  @Override
  protected void interrupted() {
    end();
  }
}
