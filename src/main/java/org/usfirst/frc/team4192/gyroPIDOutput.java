package org.usfirst.frc.team4192;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Created by szydlikr on 1/24/2017.
 */
public class gyroPIDOutput implements PIDOutput {
  private AHRS ahrs;
  private PIDController pidController;
  
  private double Kp;      // Gyroscope PID constants
  private double Ki;
  private double Kd;
  private double gyroTolerance = 2.0f;
  
  public gyroPIDOutput(AHRS ahrs, double p, double i, double d) {
    setConstants(p, i, d);
    pidController = new PIDController(Kp, Ki, Kd, 0.0, ahrs, this);
    pidController.setInputRange(-180.0f, 180.0f);
    pidController.setOutputRange(-1.0, 1.0);
    pidController.setAbsoluteTolerance(gyroTolerance);
    pidController.setContinuous(true);
    pidController.disable();
  }
  
  public void setConstants(double p, double i, double d) {
    Kp = p;
    Ki = i;
    Kd = d;
  }
  
  public void setGyroTarget(double degree) {
    
  }
  
  @Override
  public void pidWrite(double output) {
    
  }
}
