package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.Turn;

/**
 * Created by Al on 2/8/2017.
 */
public class RedLeftAuton extends CommandGroup {
  public RedLeftAuton() {
    addSequential(new Turn(45, 0.01111, 0.0, 0.001), 5);
    addSequential(new Turn(-90, 0.01111, 0.0, 0.001), 5);
    addSequential(new Turn(135, 0.0002, 0.0, 0.003), 5);
  }
}
