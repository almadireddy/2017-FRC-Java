package org.usfirst.frc.team4192;

import com.ctre.CANTalon;

/**
 * Created by aahladmadireddy on 2/25/17.
 */
public class JankoDrive {
  CANTalon leftMaster, leftSlave, rightMaster, rightSlave;
  public JankoDrive(CANTalon leftMaster, CANTalon leftSlave, CANTalon rightMaster, CANTalon rightSlave) {
    this.leftMaster = leftMaster;
    this.rightMaster = rightMaster;
    this.leftSlave = leftSlave;
    this.rightSlave = rightSlave;
  }
}
