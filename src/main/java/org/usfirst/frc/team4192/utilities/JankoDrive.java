package org.usfirst.frc.team4192.utilities;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * Created by aahladmadireddy on 2/25/17.
 */
public class JankoDrive extends RobotDrive implements PIDOutput {
  private CANTalon leftMaster, leftSlave, rightMaster, rightSlave;
  
  private double threshold = 2.0;
  
  public JankoDrive(CANTalon leftMaster, CANTalon leftSlave, CANTalon rightMaster, CANTalon rightSlave) {
    super(leftMaster, rightMaster);
    
    this.leftMaster  = leftMaster;
    this.rightMaster = rightMaster;
    this.leftSlave = leftSlave;
    this.rightSlave = rightSlave;
    
    leftMaster.setInverted(false);
    rightMaster.setInverted(false);
  
    leftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
    rightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
  
    leftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
    leftSlave.set(leftMaster.getDeviceID());
    rightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
    rightSlave.set(rightMaster.getDeviceID());
    
    /* set the peak and nominal outputs, 12V means full */
    leftMaster.configNominalOutputVoltage(+0.0f, -0.0f);
    leftMaster.configPeakOutputVoltage(+12.0f, -12.0f);
    rightMaster.configNominalOutputVoltage(+0.0f, -0.0f);
    rightMaster.configPeakOutputVoltage(+12.0f, -12.0f);
    
    setExpiration(0.1);
  }
  
  public void setSlewRate(double rampRate) {
    leftMaster.setVoltageRampRate(rampRate);
    rightMaster.setVoltageRampRate(rampRate);
  }
  
  public void prepareForAuton() {
    leftMaster.changeControlMode(CANTalon.TalonControlMode.Position);
    rightMaster.changeControlMode(CANTalon.TalonControlMode.Position);
  }
  
  public void prepareForTeleop() {
    leftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    rightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
  }
  
  public void setPID(double p, double i, double d) {
    leftMaster.setPID(p, i, d);
    rightMaster.setPID(p, i, d);
  }
  
  public void set(double setpoint) {
    leftMaster.set(setpoint);
    rightMaster.set(setpoint);
  }
  
  public void set(DriveSignal signal) {
    leftMaster.set(signal.leftMotor);
    rightMaster.set(signal.rightMotor);
  }
  
  public boolean isOnTarget() {
    return (leftMaster.getClosedLoopError() < threshold) && (rightMaster.getClosedLoopError() < threshold);
  }
  
  public double getThreshold() {
    return threshold;
  }
  
  public void setThreshold(double threshold) {
    this.threshold = threshold;
  }
  
  /**
   * Set the output to the value calculated by PIDController.
   *
   * @param output the value calculated by PIDController
   */
  @Override
  public void pidWrite(double output) {
    set(output);
  }
}
