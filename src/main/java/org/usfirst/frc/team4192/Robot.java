package org.usfirst.frc.team4192;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by Al on 1/22/2017.
 */
public class Robot extends IterativeRobot implements PIDOutput {
  private CANTalon frontLeft;
  private CANTalon frontRight;
  private CANTalon rearLeft;
  private CANTalon rearRight;
  private CANTalon flywheelLeft;
  private CANTalon flywheelRight;
  
  CANTalon.FeedbackDeviceStatus driveLeftEncoderStatus;
  boolean driveLeftEncoderExists;
  
  CANTalon.FeedbackDeviceStatus driveRightEncoderStatus;
  boolean driveRightEncoderExists;
  
  private RobotDrive drive;
  private Joystick joystick;
  private AHRS ahrs;          // the NavX board, I'm calling it AHRS because thats what all the examples call it.
  
  private double gyroKp;      // Gyroscope PID constants
  private double gyroKi;
  private double gyroKd;
  private double gyroTolerance = 2.0f;
  
  private double driveKp;     // Drive PID constants
  private double driveKi;
  private double drivekd;
  
  private double flywheelKp;  // Drive PID constants
  private double flywheelKi;
  private double flywheelKd;
  private double flywheelKf;
  
  private PIDController turnController;
  private double rotateToAngleRate;
  
  private Timer autonTimer;
  
  private boolean gyroExists = false;
  
  // updates all the flywheel pid constants to what they are on the dashboard
  private void updateFlywheelConstants() {
    flywheelKp = Double.parseDouble(SmartDashboard.getData("flywheelP").toString());
    flywheelKi = Double.parseDouble(SmartDashboard.getData("flywheelI").toString());
    flywheelKd = Double.parseDouble(SmartDashboard.getData("flywheelD").toString());
    flywheelKf = Double.parseDouble(SmartDashboard.getData("flywheelF").toString());
  }
  
  private void updateDriveConstants() {
    driveKp = Double.parseDouble(SmartDashboard.getData("driveP").toString());
    driveKi = Double.parseDouble(SmartDashboard.getData("driveI").toString());
    drivekd = Double.parseDouble(SmartDashboard.getData("driveD").toString());
  }
  
  private void updateGyroConstants() {
    gyroKp = Double.parseDouble(SmartDashboard.getData("gyroP").toString());
    gyroKi = Double.parseDouble(SmartDashboard.getData("gyroI").toString());
    gyroKd = Double.parseDouble(SmartDashboard.getData("gyroD").toString());
  }
  
  // updates all the pid constants for driving, gyroscope, and flywheel from dashboard.
  private void updatePIDConstants() {
    updateDriveConstants();
    updateGyroConstants();
    updateFlywheelConstants();
  }
  
  private void readyDriveTalonsForAuton() {
    frontLeft.changeControlMode(CANTalon.TalonControlMode.Position);
    frontRight.changeControlMode(CANTalon.TalonControlMode.Position);
    frontLeft.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
    frontRight.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
    
    /* set the peak and nominal outputs, 12V means full */
    frontLeft.configNominalOutputVoltage(+0.0f, -0.0f);
    frontRight.configNominalOutputVoltage(+0.0f, -0.0f);
    frontLeft.configPeakOutputVoltage(+12.0f, 0.0f);
    frontRight.configPeakOutputVoltage(+12.0f, 0.0f);
    
    frontLeft.setAllowableClosedLoopErr(0);
    frontRight.setAllowableClosedLoopErr(0);
    
    updateDriveConstants();
    frontLeft.setPID(driveKp, driveKi, drivekd);
    frontRight.setPID(driveKp, driveKi, drivekd);
  }
  
  //TODO: math to turn inches into encoder ticks for easier programming
  private void setDrivePosition(double distanceInInches) {
    double distanceInTicks = distanceInInches*0;    //replace 0 with some calculated factor
    frontLeft.setPosition(distanceInTicks);
    frontRight.setPosition(distanceInTicks);
  }
  
  /**
   * @param angle the angle you want to turn to, from -180 to 180
   */
  private void rotateToAngle(double angle) {
    SmartDashboard.putNumber("targetHeading", angle);
    turnController.enable();
    turnController.setSetpoint(angle);
    
/*  boolean finishedTurning = false;
    int targetChecker = 0;
    
    while (!finishedTurning) {
      drive.arcadeDrive(0.0, rotateToAngleRate);
      if (turnController.onTarget())
        targetChecker++;          // targetChecker should increase once every 20 milliseconds approximately
      
      if (targetChecker == 75)    // 75 would be approximately 1.5 seconds, so robot has to maintain angle for 1.5 seconds before pid stops
        finishedTurning = true;
    }  */

    while (!turnController.onTarget()) {
      SmartDashboard.putNumber("actualHeading", ahrs.getAngle());
      try {
        wait(50);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    turnController.disable();
  }
  
  @Override
  public void robotInit() {
    frontLeft = new CANTalon(0);                // make CAN Talon SRX objects
    frontRight = new CANTalon(1);
    rearLeft = new CANTalon(2);
    rearRight = new CANTalon(3);
    flywheelLeft = new CANTalon(4);
    flywheelRight = new CANTalon(5);
    
    frontLeft.setInverted(true);   // These might not need to be inverted.
    frontRight.setInverted(true);
  
    rearLeft.changeControlMode(CANTalon.TalonControlMode.Follower);   // switch the rear motors to slaves
    rearRight.changeControlMode(CANTalon.TalonControlMode.Follower);
    rearLeft.set(0);                                                  // point slaves to their master device id's
    rearRight.set(1);
    
    drive = new RobotDrive(frontLeft, frontRight);
    drive.setExpiration(0.1);
    
    joystick = new Joystick(0);
    
    autonTimer = new Timer();
  
    driveLeftEncoderStatus = frontLeft.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
    driveLeftEncoderExists = CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent == driveLeftEncoderStatus;
    driveRightEncoderStatus = frontRight.isSensorPresent(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
    driveRightEncoderExists = CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent == driveRightEncoderStatus;
  
    try {
      ahrs = new AHRS(SPI.Port.kMXP); // set the NavX board to use the MXP port in the middle of the roboRIO
      gyroExists = true;
      SmartDashboard.putBoolean("gyroPIDExists", true);
    }
    catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
      gyroExists = false;
      SmartDashboard.putBoolean("gyroPIDExists", false);
    }
    
    updatePIDConstants();
    
    if (gyroExists) {
      turnController = new PIDController(gyroKp, gyroKi, gyroKd, ahrs, this);
      turnController.setInputRange(-180.0f, 180.0f);
      turnController.setOutputRange(-1.0, 1.0);
      turnController.setAbsoluteTolerance(gyroTolerance);
      turnController.setContinuous(true);
      turnController.disable();
    } else {
      System.out.println("GYRO PID DOES NOT EXIST");
      SmartDashboard.putBoolean("gyroPIDExists", false);
    }
  }
  
  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
  }
  
  @Override
  public void autonomousInit() {
    ahrs.reset();   // reset the gyro board
    autonTimer.reset();
    autonTimer.start();
  }
  
  @Override
  public void autonomousPeriodic() {
    
  }
  
  @Override
  public void teleopInit() {
    super.teleopInit();
  }
  
  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(joystick.getY(), joystick.getX(), true);  // if the motors don't need to be inverted, add negatives to the axes.
    updateFlywheelConstants();
  }
  
  @Override
  public void pidWrite(double output) {
    rotateToAngleRate = output;
  }
}

