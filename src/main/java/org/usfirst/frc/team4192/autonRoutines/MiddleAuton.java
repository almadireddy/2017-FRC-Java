package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.ActuatorBackward;
import org.usfirst.frc.team4192.commands.ActuatorForward;
import org.usfirst.frc.team4192.commands.ActuatorInitialize;
import org.usfirst.frc.team4192.commands.driveOn;

/**
 * Created by a Al on 2/8/2017.
 */
public class MiddleAuton extends CommandGroup {
  public MiddleAuton() {
    addParallel(new ActuatorInitialize(), 10);
    addSequential(new driveOn(80), 4);
    addSequential(new ActuatorForward(), 1.5);
    addSequential(new ActuatorBackward(), 1.5);
  }
}
