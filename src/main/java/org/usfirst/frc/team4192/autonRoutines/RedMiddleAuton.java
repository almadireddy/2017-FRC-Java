package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.driveOn;

/**
 * Created by a faggot on 2/8/2017.
 */
public class RedMiddleAuton extends CommandGroup {
  public RedMiddleAuton() {
    addSequential(new driveOn(120), 50);
  }
}
