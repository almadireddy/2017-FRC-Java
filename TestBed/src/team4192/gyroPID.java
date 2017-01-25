package team4192;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Created by Al on 1/24/2017.
 */
public class gyroPID implements PIDOutput {
  private SpeedController left, right;

  public gyroPID(SpeedController leftSide, SpeedController rightSide) {
    left = leftSide;
    right = rightSide;
  }
  
  @Override
  public void pidWrite(double output) {
    left.set(output);
    right.set(-output);
  }
}
