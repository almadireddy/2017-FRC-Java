package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.ActuatorInitialize;
import org.usfirst.frc.team4192.commands.Move;

/**
 * Created by Al on 3/9/2017.
 */
public class DefaultAuton extends CommandGroup {
  public DefaultAuton() {
    addParallel(new ActuatorInitialize(), 3);
    addSequential(new Move(180));
  }
}
