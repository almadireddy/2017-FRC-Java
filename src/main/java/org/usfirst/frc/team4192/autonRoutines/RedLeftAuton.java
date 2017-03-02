package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.Turn;

/**
 * Created by Al on 2/8/2017.
 */
public class RedLeftAuton extends CommandGroup {
  public RedLeftAuton() {
    addSequential(new Turn(45), 50);
  }
}
