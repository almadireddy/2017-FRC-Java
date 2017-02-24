package org.usfirst.frc.team4192;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4192.autonRoutines.*;
import org.usfirst.frc.team4192.utilities.gyroPID;

/**
 * Created by Al on 1/22/2017.
 */
public class Robot extends IterativeRobot {
  public static CANTalon frontLeft;
  public static CANTalon frontRight;
  public static CANTalon flywheel;
  public static CANTalon lift;
  public static CANTalon intake;
  public static CANTalon agitator;
  
  private static int drivePIDThreshold = 10;
  
  private RobotDrive drive;
  private Joystick joystick;
  private AHRS ahrs;          // the NavX board, I'm calling it ahrs because thats what all the examples call it.
  
  private static double gyroKp;      // Gyroscope PID constants
  private static double gyroKi;
  private static double gyroKd;
  private static double gyroTolerance = 2.0f;
  
  private static double driveKp;     // Drive PID constants
  private static double driveKi;
  private static double drivekd;
  
  private static double flywheelKp;  // Drive PID constants
  private static double flywheelKi;
  private static double flywheelKd;
  private static double flywheelKf;
  private static double flywheelTargetRPM;
  
  public static PIDController turnController;
  private boolean gyroExists = false;
  private gyroPID gyroPID;
    
  /// Start Autonomous Stuff ///
  private RedLeftAuton redLeftAuton;
  private RedMiddleAuton redMiddleAuton;
  private RedRightAuton redRightAuton;
  
  private BlueLeftAuton blueLeftAuton;
  private BlueMiddleAuton blueMiddleAuton;
  private BlueRightAuton blueRightAuton;
  /// End Autonomous Stuff ///
  
  
  private static void setDriveConstants() {
    frontLeft.setPID(driveKp, driveKi, drivekd);
    frontRight.setPID(driveKp, driveKi, drivekd);
  }
  
  private static void setGyroConstants() {
    turnController.setPID(gyroKp, gyroKi, gyroKd);
  }
  
  private static void setFlywheelConstants() {
    flywheel.setP(flywheelKp);
    flywheel.setI(flywheelKi);
    flywheel.setD(flywheelKd);
    flywheel.setF(flywheelKf);
  }
    
  // updates all the drive pid constants to what they are on the dashboard
  public static void updateDriveConstants() {
    driveKp = Double.parseDouble(SmartDashboard.getData("driveP").toString());
    driveKi = Double.parseDouble(SmartDashboard.getData("driveI").toString());
    drivekd = Double.parseDouble(SmartDashboard.getData("driveD").toString());
    setDriveConstants();
  }
  
  // updates all the gyro pid constants to what they are on the dashboard
  public static void updateGyroConstants() {
    gyroKp = Double.parseDouble(SmartDashboard.getData("gyroP").toString());
    gyroKi = Double.parseDouble(SmartDashboard.getData("gyroI").toString());
    gyroKd = Double.parseDouble(SmartDashboard.getData("gyroD").toString());
    setGyroConstants();
  }
  
  // updates all the flywheel pid constants to what they are on the dashboard
  public static void updateFlywheelConstants() {
    flywheelKp = Double.parseDouble(SmartDashboard.getData("flywheelP").toString());
    flywheelKi = Double.parseDouble(SmartDashboard.getData("flywheelI").toString());
    flywheelKd = Double.parseDouble(SmartDashboard.getData("flywheelD").toString());
    flywheelKf = Double.parseDouble(SmartDashboard.getData("flywheelF").toString());
    setFlywheelConstants();
  }
  
  // calls the three constants update functions
  public static void updatePIDConstants() {
    updateDriveConstants();
    updateGyroConstants();
    updateFlywheelConstants();
  }
  
  public static void setFlywheelTargetRPM() {
    flywheel.setSetpoint(flywheelTargetRPM);
  }
  
  public static void updateFlywheelTargetRPM() {
    flywheelTargetRPM = Double.parseDouble(SmartDashboard.getData("targetRPMControl").toString());
    setFlywheelTargetRPM();
  }
  
  public static void switchDriveMotorsToPositionControl() {
    frontLeft.changeControlMode(CANTalon.TalonControlMode.Position);
    frontRight.changeControlMode(CANTalon.TalonControlMode.Position);
    
    /* set the peak and nominal outputs, 12V means full */
    frontLeft.configNominalOutputVoltage(+0.0f, -0.0f);
    frontRight.configNominalOutputVoltage(+0.0f, -0.0f);
    frontLeft.configPeakOutputVoltage(+12.0f, -12.0f);
    frontRight.configPeakOutputVoltage(+12.0f, -12.0f);
    
    frontLeft.setAllowableClosedLoopErr(drivePIDThreshold);
    frontRight.setAllowableClosedLoopErr(drivePIDThreshold);
    
    updateDriveConstants();
  }
  
  public static void switchDriveMotorsToDefaultControl() {
    frontLeft.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    frontRight.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    
    frontLeft.configNominalOutputVoltage(+0.0f, -0.0f);
    frontRight.configNominalOutputVoltage(+0.0f, -0.0f);
    frontLeft.configPeakOutputVoltage(+12.0f, -12.0f);
    frontRight.configPeakOutputVoltage(+12.0f, -12.0f);
  }
    
  public static boolean driveOnTarget() {
    return (frontLeft.getClosedLoopError() < drivePIDThreshold) && (frontRight.getClosedLoopError() < drivePIDThreshold);
  }

  public static boolean gyroOnTarget() {
    return turnController.onTarget();
  }
  
  @Override
  public void robotInit() {
    frontLeft = new CANTalon(1);                // make CAN Talon SRX objects
    frontRight = new CANTalon(2);
  
    CANTalon rearLeft = new CANTalon(3);
    CANTalon rearRight = new CANTalon(4);

    flywheel = new CANTalon(5);
    lift = new CANTalon(6);
    intake = new CANTalon(7);
    agitator = new CANTalon(8);
    
    frontLeft.setInverted(true);   // These might not need to be inverted.
    frontRight.setInverted(true);
    
    frontLeft.setVoltageRampRate(12);
    frontRight.setVoltageRampRate(12);
  
    frontLeft.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
    frontRight.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
  
    rearLeft.changeControlMode(CANTalon.TalonControlMode.Follower);   // switch the rear motors to slaves
    rearLeft.set(1);                                                  // point slaves to their master device id's
    rearRight.changeControlMode(CANTalon.TalonControlMode.Follower);
    rearRight.set(2);
    
    flywheel.changeControlMode(CANTalon.TalonControlMode.Speed);
    flywheel.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder);
    flywheel.setProfile(0);
    
    drive = new RobotDrive(frontLeft, frontRight);
    drive.setExpiration(0.1);
    
    joystick = new Joystick(0);
    
    try {
      ahrs = new AHRS(SPI.Port.kMXP); // set the NavX board to use the MXP port in the middle of the roboRIO
      gyroExists = true;
      DriverStation.reportWarning("instantiated navX MXP:  ", false);
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
      turnController = new PIDController(0.01, 0.0, 0, ahrs, gyroPID);
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
    
    redLeftAuton = new RedLeftAuton();
    redMiddleAuton = new RedMiddleAuton();
    redRightAuton = new RedRightAuton();
    blueLeftAuton = new BlueLeftAuton();
    blueMiddleAuton = new BlueMiddleAuton();
    blueRightAuton = new BlueRightAuton();

    // A new thread to continually update flywheel constants and target RPM from Driverstation.
    // This way, if driverstation is slow to send a package, then the teleopPeriodic
    // function will not get hung up and flywheel will continue at the last set RPM and constants.
    Thread flywheelControlThread = new Thread(() -> {
      while (!Thread.interrupted()) {
        updateFlywheelConstants();
        updateFlywheelTargetRPM();
      }
    });
    flywheelControlThread.start();
    
    // Another thread that handles updating all the values on driverstation so that teleopPeriodic
    // doesn't get hung up in the case of slow network speeds.
    Thread dashboardUpdateThread = new Thread(() -> {
      while (!Thread.interrupted()) {
        SmartDashboard.putNumber("actualHeading", ahrs.getAngle());
        SmartDashboard.putNumber("leftActualRPM", flywheel.getEncVelocity());
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
    
    switch (SmartDashboard.getData("Selected Autonomous").toString()) {
      case "Red Left":
        redLeftAuton.start();
        break;
        
      case "Red Middle":
        redMiddleAuton.start();
        break;
      
      case "Red Right":
        redRightAuton.start();
        break;
      
      case "Blue Left":
        blueLeftAuton.start();
        break;
      
      case "Blue Middle":
        blueMiddleAuton.start();
        break;
        
      case "Blue Right":
        blueRightAuton.start();
        break;
    }
  }
  
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }
  
  @Override
  public void teleopInit() {
    ahrs.reset();       // reset the gyro board
    if (turnController.isEnabled())
      turnController.disable();
    
    switchDriveMotorsToDefaultControl();
    Scheduler.getInstance().removeAll();
  }
  
  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(joystick.getY(), joystick.getX(), true);  // if the motors don't need to be inverted, add negatives to the axes.
  }
}