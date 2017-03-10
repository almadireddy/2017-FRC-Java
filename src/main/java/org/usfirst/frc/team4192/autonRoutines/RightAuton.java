package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.Turn;
import org.usfirst.frc.team4192.commands.driveOn;

/**
 * Created by Al on 2/8/2017.
 */
public class RightAuton extends CommandGroup {
  public RightAuton() {
    addSequential(new driveOn(8*12), 5);
    addSequential(new Turn(-45), 3);
    addSequential(new driveOn(36), 5);
  }
}
