package org.usfirst.frc.team4192;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by Al on 1/22/2017.
 */
public class Robot extends IterativeRobot {
  private CANTalon frontLeft;
  private CANTalon frontRight;
  private CANTalon rearLeft;
  private CANTalon rearRight;
  private CANTalon flywheelLeft;
  private CANTalon flywheelRight;
  
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
  private double flywheelTargetRPM;
  
  private PIDController turnController;
  private double rotateToAngleRate;
  private boolean gyroExists = false;
  private gyroPID gyroPID;
  
  private Timer autonTimer;
  

  // updates all the drive pid constants to what they are on the dashboard
  private void updateDriveConstants() {
    driveKp = Double.parseDouble(SmartDashboard.getData("driveP").toString());
    driveKi = Double.parseDouble(SmartDashboard.getData("driveI").toString());
    drivekd = Double.parseDouble(SmartDashboard.getData("driveD").toString());
  }
  
  // updates all the gyro pid constants to what they are on the dashboard
  private void updateGyroConstants() {
    gyroKp = Double.parseDouble(SmartDashboard.getData("gyroP").toString());
    gyroKi = Double.parseDouble(SmartDashboard.getData("gyroI").toString());
    gyroKd = Double.parseDouble(SmartDashboard.getData("gyroD").toString());
  }
  
  // calls the three constants update functions
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
  
  private void readyDriveTalonsForTeleop() {
    frontLeft.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    frontRight.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    frontLeft.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
    frontRight.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
  
    frontLeft.configNominalOutputVoltage(+0.0f, -0.0f);
    frontRight.configNominalOutputVoltage(+0.0f, -0.0f);
    frontLeft.configPeakOutputVoltage(+12.0f, -12.0f);
    frontRight.configPeakOutputVoltage(+12.0f, -12.0f);
  }
  
  private void setFlywheelTargetRPM() {
    flywheelLeft.setSetpoint(flywheelTargetRPM);
    flywheelRight.setSetpoint(flywheelTargetRPM);
  }
  
  private void setFlywheelConstants() {
    flywheelLeft.setP(flywheelKp);
    flywheelLeft.setI(flywheelKi);
    flywheelLeft.setD(flywheelKd);
    flywheelLeft.setF(flywheelKf);
    flywheelRight.setP(flywheelKp);
    flywheelRight.setI(flywheelKi);
    flywheelRight.setD(flywheelKd);
    flywheelRight.setF(flywheelKf);
  }
  
  // updates all the flywheel pid constants to what they are on the dashboard
  private void updateFlywheelConstants() {
    flywheelKp = Double.parseDouble(SmartDashboard.getData("flywheelP").toString());
    flywheelKi = Double.parseDouble(SmartDashboard.getData("flywheelI").toString());
    flywheelKd = Double.parseDouble(SmartDashboard.getData("flywheelD").toString());
    flywheelKf = Double.parseDouble(SmartDashboard.getData("flywheelF").toString());
    setFlywheelConstants();
  }
  
  private void updateFlywheelTargetRPM() {
    flywheelTargetRPM = Double.parseDouble(SmartDashboard.getData("targetRPMControl").toString());
    setFlywheelTargetRPM();
  }
  
  //TODO: math to turn inches into encoder ticks for easier programming
  private void setDriveTarget(double distanceInInches) {
    double distanceInTicks = distanceInInches*0;    //replace 0 with some calculated factor
    frontLeft.setPosition(distanceInTicks);
    frontRight.setPosition(distanceInTicks);
  }
  
  /**
   * @param angle the angle you want to turn to, from -180 to 180
   */
  private void rotateToAngle(double angle) {
    SmartDashboard.putNumber("targetHeading", angle);
    SmartDashboard.putNumber("actualHeading", ahrs.getAngle());
    turnController.enable();
    turnController.setSetpoint(angle);
    if (turnController.onTarget())
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
    
    flywheelLeft.changeControlMode(CANTalon.TalonControlMode.Speed);
    flywheelRight.changeControlMode(CANTalon.TalonControlMode.Speed);
    flywheelLeft.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder);
    flywheelRight.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder);
    
    flywheelLeft.setProfile(0);
    flywheelRight.setProfile(0);
    
    flywheelRight.setInverted(true); // needs to be inverted so pid can take positive value but spins counter-clockwise (outwards)
    
    drive = new RobotDrive(frontLeft, frontRight);
    drive.setExpiration(0.1);
    
    joystick = new Joystick(0);
    
    autonTimer = new Timer();
  
    try {
      ahrs = new AHRS(SPI.Port.kMXP); // set the NavX board to use the MXP port in the middle of the roboRIO
      gyroExists = true;
      SmartDashboard.putBoolean("gyroPIDExists", true);
    }
    // if gyro doesnt initialize correctly, report it and set gyroexists to false. this is so that the robot doesnt crash because of an exception
    catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
      gyroExists = false;
      SmartDashboard.putBoolean("gyroPIDExists", false);
    }
    
    if (gyroExists) {
      gyroPID = new gyroPID(frontLeft, frontRight);
      turnController = new PIDController(gyroKp, gyroKi, gyroKd, ahrs, gyroPID);
      turnController.setInputRange(-180.0f, 180.0f);
      turnController.setOutputRange(-1.0, 1.0);
      turnController.setAbsoluteTolerance(gyroTolerance);
      turnController.setContinuous(true);
      turnController.disable();
    } else {
      System.out.println("GYRO PID DOES NOT EXIST");
      SmartDashboard.putBoolean("gyroPIDExists", false);
    }
  
    updatePIDConstants();
    updateFlywheelTargetRPM();
    
    // A new thread to continually update flywheel constants and target RPM from Driverstation.
    // This way, if driverstation is slow to send a package, then the teleopPeriodic
    // function will not get hung up and flywheel will continue at the last set RPM and constants.
    Thread flywheelControlThread = new Thread(() -> {
      while (!Thread.interrupted()) {
        updateFlywheelTargetRPM();
        updateFlywheelConstants();
        flywheelLeft.setSetpoint(flywheelTargetRPM);
        flywheelRight.setSetpoint(flywheelTargetRPM);
      }
    });
    flywheelControlThread.start();
    
    
    // Another thread that handles updating all the values on driverstation so that teleopPeriodic
    // doesn't get hung up in the case of slow network speeds.
    Thread dashboardUpdateThread = new Thread(() -> {
      while (!Thread.interrupted()) {
        SmartDashboard.putNumber("actualHeading", ahrs.getAngle());
        SmartDashboard.putNumber("leftActualRPM", flywheelLeft.getEncVelocity());
        SmartDashboard.putNumber("rightActualRPM", flywheelRight.getEncVelocity());
        SmartDashboard.putNumber("targetRPM", flywheelTargetRPM);
      }
    });
    dashboardUpdateThread.start();
  }
  
  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
  }
  
  @Override
  public void autonomousInit() {
    ahrs.reset();       // reset the gyro board
    readyDriveTalonsForAuton();
    
    autonTimer.reset();
    autonTimer.start();
  }
  
  @Override
  public void autonomousPeriodic() {
    
  }
  
  @Override
  public void teleopInit() {
    readyDriveTalonsForTeleop();
  }
  
  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(joystick.getY(), joystick.getX(), true);  // if the motors don't need to be inverted, add negatives to the axes.
  }
}

