package org.usfirst.frc.team4192.autonRoutines;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.Turn;
import org.usfirst.frc.team4192.commands.driveOn;

/**
 * Created by Al on 2/8/2017.
 */
public class RightAuton extends CommandGroup {
  public RightAuton() {
    addSequential(new driveOn(75), 3.5);
    addSequential(new Turn(-70), 2);
    addSequential(new driveOn(46), 5);
  }
}
