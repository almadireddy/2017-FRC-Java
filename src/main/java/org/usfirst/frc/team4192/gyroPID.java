package org.usfirst.frc.team4192;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Created by Al on 1/24/2017.
 */
public class gyroPID implements PIDOutput {
  private CANTalon left, right;

  public gyroPID(CANTalon leftSide, CANTalon rightSide) {
    left = leftSide;
    right = rightSide;
  }
  
  @Override
  public void pidWrite(double output) {
    left.set(output);
    right.set(-output);
  }
}