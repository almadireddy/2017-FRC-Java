package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.ActuatorInitialize;
import org.usfirst.frc.team4192.commands.driveOn;

/**
 * Created by darkb on 3/9/2017.
 */
public class DefaultAuton extends CommandGroup {
  public DefaultAuton() {
    addParallel(new ActuatorInitialize(), 5);
    addSequential(new driveOn(100));
  }
}
