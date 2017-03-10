package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.Turn;
import org.usfirst.frc.team4192.commands.driveOn;

/**
 * Created by Al on 2/8/2017.
 */
public class LeftAuton extends CommandGroup {
  public LeftAuton() {
    addSequential(new driveOn(92), 3.5);
    addSequential(new Turn(63), 3.5);
    addSequential(new driveOn(34), 5);
  }
}
