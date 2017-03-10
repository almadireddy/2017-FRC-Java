package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.driveOn;

/**
 * Created by a Al on 2/8/2017.
 */
public class MiddleAuton extends CommandGroup {
  public MiddleAuton() {
    addSequential(new driveOn(8*12 -24), 10);
  }
}
