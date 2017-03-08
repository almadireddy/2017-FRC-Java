package org.usfirst.frc.team4192.utilities;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    
    leftMaster.enableBrakeMode(true);
    leftSlave.enableBrakeMode(true);
    rightMaster.enableBrakeMode(true);
    rightSlave.enableBrakeMode(true);
  
    leftMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
    rightMaster.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
  
    leftMaster.reverseOutput(false);
    leftMaster.reverseSensor(true);
    rightMaster.reverseOutput(true);
    
    leftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
    leftSlave.set(leftMaster.getDeviceID());
    rightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
    rightSlave.set(rightMaster.getDeviceID());
    
    /* set the peak and nominal outputs, 12V means full */
    leftMaster.configNominalOutputVoltage(+0.0f, -0.0f);
    leftSlave.configPeakOutputVoltage(+12.0f, -12.0f);
    rightMaster.configNominalOutputVoltage(+0.0f, -0.0f);
    rightSlave.configPeakOutputVoltage(+12.0f, -12.0f);
    
    updatePID();
    
    setExpiration(0.1);
  }
  
  public void setSlewRate(double rampRate) {
    leftMaster.setVoltageRampRate(rampRate);
    rightMaster.setVoltageRampRate(rampRate);
  }
  
  public void prepareForAuton() {
    leftMaster.changeControlMode(CANTalon.TalonControlMode.Position);
    rightMaster.changeControlMode(CANTalon.TalonControlMode.Position);
    
    zeroEncoders();
  }
  
  public void prepareForTeleop() {
    leftMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    rightMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    zeroEncoders();
  }
  
  public void updatePID() {
    leftMaster.setP(SmartDashboard.getNumber("driveP", 0.0));
    leftMaster.setI(SmartDashboard.getNumber("driveI", 0.0));
    leftMaster.setD(SmartDashboard.getNumber("driveD", 0.0));
  
    rightMaster.setP(SmartDashboard.getNumber("driveP", 0.0));
    rightMaster.setI(SmartDashboard.getNumber("driveI", 0.0));
    rightMaster.setD(SmartDashboard.getNumber("driveD", 0.0));
  }
  
  public void zeroEncoders() {
    leftMaster.setEncPosition(0);
    rightMaster.setEncPosition(0);
  }
  
  public void setPID(double p, double i, double d) {
    leftMaster.setPID(p, i, d);
    rightMaster.setPID(p, i, d);
  }
  
  public void set(double setpoint) {
    leftMaster.set(setpoint);
    rightMaster.set(setpoint);
  }
  
  public void setSetpoint(double setpoint) {
    leftMaster.setSetpoint(setpoint);
    rightMaster.setSetpoint(setpoint);
  }
  
  public void set(DriveSignal signal) {
    leftMaster.set(signal.leftMotor);
    rightMaster.set(signal.rightMotor);
  }
  
  public boolean isOnTarget() {
    return (leftMaster.getClosedLoopError() < threshold) && (rightMaster.getClosedLoopError() < threshold);
  }
  
  public double getLeft() {
    return leftMaster.get();
  }
  
  public double getRight() {
    return rightMaster.get();
  }
  
  public double getLeftValue() {
    return leftMaster.getEncPosition();
  }
  
  public double getRightValue() {
    return rightMaster.getEncPosition();
  }
  
  public void disable() {
    leftMaster.disable();
    rightMaster.disable();
  }
  
  public void enable() {
    leftMaster.enable();
    rightMaster.enable();
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
