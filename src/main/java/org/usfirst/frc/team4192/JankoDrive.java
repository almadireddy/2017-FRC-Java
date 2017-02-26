package org.usfirst.frc.team4192;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 * Created by aahladmadireddy on 2/25/17.
 */
public class JankoDrive extends RobotDrive {
  private CANTalon leftMaster, leftSlave, rightMaster, rightSlave;
  
  public JankoDrive(CANTalon leftMaster, CANTalon leftSlave, CANTalon rightMaster, CANTalon rightSlave) {
    super(leftMaster, rightMaster);
    
    this.leftMaster  = leftMaster;
    this.rightMaster = rightMaster;
    this.leftSlave   = leftSlave;
    this.rightSlave  = rightSlave;
    
    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
  
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
}
