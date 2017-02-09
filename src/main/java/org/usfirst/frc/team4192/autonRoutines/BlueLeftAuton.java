package org.usfirst.frc.team4192.autonRoutines;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4192.commands.driveOn;
import org.usfirst.frc.team4192.commands.turn;

/**
 * Created by Al on 2/8/2017.
 */
public class BlueLeftAuton extends CommandGroup {
  public BlueLeftAuton() {
    addSequential(new driveOn(120));
    addSequential(new turn(45));
  }
}
