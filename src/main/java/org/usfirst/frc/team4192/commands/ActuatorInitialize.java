package org.usfirst.frc.team4192.commands;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4192.Robot;

/**
 * Created by Al on 3/29/2017.
 */
public class ActuatorInitialize extends Command {
  public ActuatorInitialize() {
    super();
  }
  
  @Override
  protected void initialize() {
    Robot.actuator.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
  }
  
  @Override
  protected void execute() {
    if (Robot.backSwitch.get()) {
      Robot.actuator.set(-0.15);
    } else {
      Robot.actuator.set(0);
    }
  }
  
  @Override
  protected boolean isFinished() {
    return !Robot.backSwitch.get();
  }
  
  @Override
  protected void end() {
    Robot.actuator.set(0);
    Robot.actuator.changeControlMode(CANTalon.TalonControlMode.Position);
    Robot.actuator.setEncPosition(0);
  }
  
  @Override
  protected void interrupted() {
    end();
  }
}
