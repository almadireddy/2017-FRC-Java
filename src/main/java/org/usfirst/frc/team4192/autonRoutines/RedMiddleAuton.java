package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.Turn;
import org.usfirst.frc.team4192.commands.driveOn;

/**
 * Created by Al on 2/8/2017.
 */
public class RedMiddleAuton extends CommandGroup {
  public RedMiddleAuton() {
    addSequential(new driveOn(120));
    addSequential(new Turn(45));
  }
}
