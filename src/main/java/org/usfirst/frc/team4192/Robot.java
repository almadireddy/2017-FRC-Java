package org.usfirst.frc.team4192;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4192.autonRoutines.*;
import org.usfirst.frc.team4192.utilities.JaggernautGyroDrive;
import org.usfirst.frc.team4192.utilities.JaggernautJoystick;
import org.usfirst.frc.team4192.utilities.JankoDrive;

/**
 * Created by Al on 1/22/2017.
 */
public class Robot extends IterativeRobot {
  public static CANTalon frontLeft;
  public static CANTalon frontRight;
  private static CANTalon rearLeft;
  private static CANTalon rearRight;
  public static CANTalon flywheel;
  private static CANTalon lift;
  public static CANTalon intake;
  public static CANTalon agitator;
  
  public static JankoDrive jankoDrive;
  
  private static int drivePIDThreshold = 10;
  
  private JaggernautJoystick joystick;
  private AHRS ahrs;          // the NavX board, I'm calling it ahrs because thats what all the examples call it.
  
  private static double gyroKp;      // Gyroscope PID constants
  private static double gyroKi;
  private static double gyroKd;
  
  private static double driveKp;     // Drive PID constants
  private static double driveKi;
  private static double drivekd;
  
  private static double flywheelKp;  // Drive PID constants
  private static double flywheelKi;
  private static double flywheelKd;
  private static double flywheelKf;
  private static double flywheelTargetRPM;
  
  public static PIDController turnController;
  private JaggernautGyroDrive jaggernautGyroDrive;
  
  
  private RedLeftAuton redLeftAuton;
  private RedMiddleAuton redMiddleAuton;
  private RedRightAuton redRightAuton;
  
  private BlueLeftAuton blueLeftAuton;
  private BlueMiddleAuton blueMiddleAuton;
  private BlueRightAuton blueRightAuton;
  
  private NetworkTable table;
  
  ////// End Instance Variables //////
  
  private void setDriveConstants() {
    jankoDrive.setPID(drivekd, driveKi, drivekd);
  }
  
  private void setGyroConstants() {
    turnController.setPID(gyroKp, gyroKi, gyroKd);
  }
  
  private void setFlywheelConstants() {
    flywheel.setP(flywheelKp);
    flywheel.setI(flywheelKi);
    flywheel.setD(flywheelKd);
    flywheel.setF(flywheelKf);
  }
  
  // updates all the drive pid constants to what they are on the dashboard
  private void updateDriveConstants() {
    driveKp = SmartDashboard.getNumber("driveP", JankoConstants.defaultDriveKp);
    driveKi = SmartDashboard.getNumber("driveI", JankoConstants.defaultDriveKi);
    drivekd = SmartDashboard.getNumber("driveD", JankoConstants.defaultDriveKd);
    setDriveConstants();
  }
  
  // updates all the gyro pid constants to what they are on the dashboard
  private void updateGyroConstants() {
    gyroKp = SmartDashboard.getNumber("gyroP", JankoConstants.defaultGyroKp);
    gyroKi = SmartDashboard.getNumber("gyroI", JankoConstants.defaultGyroKi);
    gyroKd = SmartDashboard.getNumber("gyroD", JankoConstants.defaultGyroKd);
    setGyroConstants();
  }
  
  // updates all the flywheelID pid constants to what they are on the dashboard
  private void updateFlywheelConstants() {
    flywheelKp = SmartDashboard.getNumber("flywheelP", JankoConstants.defaultFlywheelKp);
    flywheelKi = SmartDashboard.getNumber("flywheelI", JankoConstants.defaultFlywheelKi);
    flywheelKd = SmartDashboard.getNumber("flywheelD", JankoConstants.defaultFlywheelKd);
    flywheelKf = SmartDashboard.getNumber("flywheelF", JankoConstants.defaultFlywheelKf);
    setFlywheelConstants();
  }
  
  // calls the three constants update functions
  private void updatePIDConstants() {
    updateDriveConstants();
    updateGyroConstants();
    updateFlywheelConstants();
  }
  
  private void setFlywheelTargetRPM() {
    flywheel.setSetpoint(flywheelTargetRPM);
  }
  
  private void updateFlywheelTargetRPM() {
    flywheelTargetRPM = SmartDashboard.getNumber("targetRPMControl", 0.0);
    setFlywheelTargetRPM();
  }
  
  private void zeroSensors() {
    ahrs.reset();
  }
  
  @Override
  public void robotInit() {
    frontLeft = new CANTalon(JankoConstants.frontLeftID);
    frontRight = new CANTalon(JankoConstants.frontRightID);
    rearLeft  = new CANTalon(JankoConstants.rearLeftID);
    rearRight = new CANTalon(JankoConstants.rearRightID);
    
    table = NetworkTable.getTable("Autonomous Boolean");
    
    jankoDrive = new JankoDrive(frontLeft, rearLeft, frontRight, rearRight);
    jankoDrive.setSlewRate(12);

    flywheel = new CANTalon(JankoConstants.flywheelID);
    lift = new CANTalon(JankoConstants.liftID);
    intake = new CANTalon(JankoConstants.intakeID);
    agitator = new CANTalon(JankoConstants.agitatorID);
    
    flywheel.changeControlMode(CANTalon.TalonControlMode.Speed);
    flywheel.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder);
    flywheel.setProfile(0);
    
    joystick = new JaggernautJoystick(JankoConstants.joystick);
    
    ahrs = new AHRS(SPI.Port.kMXP); // set the NavX board to use the MXP port in the middle of the roboRIO
    DriverStation.reportWarning("instantiated navX MXP:  ", false);
    SmartDashboard.putBoolean("gyroPIDExists", true);
    
    jaggernautGyroDrive = new JaggernautGyroDrive(frontLeft, frontRight);
    turnController = new PIDController(0.01, 0.0, 0, ahrs, jaggernautGyroDrive);
    turnController.setInputRange(-360.0f, 360.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(2.0);
    turnController.setContinuous(false);
    turnController.disable();

    updatePIDConstants();
    
    redLeftAuton = new RedLeftAuton();
    redMiddleAuton = new RedMiddleAuton();
    redRightAuton = new RedRightAuton();
    blueLeftAuton = new BlueLeftAuton();
    blueMiddleAuton = new BlueMiddleAuton();
    blueRightAuton = new BlueRightAuton();

    Thread flywheelControlThread = new Thread(() -> {
      while (!Thread.interrupted()) {
        updateFlywheelConstants();
        updateFlywheelTargetRPM();
      }
    });
    flywheelControlThread.start();
    
    Thread dashboardUpdateThread = new Thread(() -> {
      while (!Thread.interrupted()) {
        SmartDashboard.putNumber("actualHeading", ahrs.getAngle());
        SmartDashboard.putNumber("leftActualRPM", flywheel.getEncVelocity());
        SmartDashboard.putNumber("targetRPM", flywheelTargetRPM);
      }
    });
    dashboardUpdateThread.start();
    
//    CameraServer.getInstance().startAutomaticCapture("frontCamera", "http://10.41.92.165:8080/?action-stream");
  }
  
  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
    table.putBoolean("TeleopPeriodic", false);
  }
  
  @Override
  public void autonomousInit() {
    zeroSensors();
    jankoDrive.prepareForAuton();
    updateGyroConstants();
    
    switch (SmartDashboard.getString("Selected Autonomous", "default")) {
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
      case "default":
        break;
      default:
        break;
    }
  }
  
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }
  
  private void intakeControl() {
    if (joystick.isHeldDown(6))
      intake.set(1);
    else if (joystick.isHeldDown(7))
      intake.set(-1);
    else
      intake.set(0);
  }
  
  private void flywheelControl() {
    if (joystick.buttonPressed(JankoConstants.flywheelToggle)) {
      if (flywheel.isEnabled())
        flywheel.disable();
      else {
        flywheel.enable();
        flywheel.set(flywheelTargetRPM);
      }
    }
  }
  
  @Override
  public void teleopInit() {
    zeroSensors();
    
    if (turnController.isEnabled())
      turnController.disable();
    
    jankoDrive.prepareForTeleop();
    table.putBoolean("TeleopInit", true);
  }
  
  @Override
  public void teleopPeriodic() {
    jankoDrive.arcadeDrive(joystick.getYaxis(), joystick.getXaxis(), true);
    joystick.update();
    intakeControl();
    flywheelControl();
    table.putBoolean("TeleopPeriodic", true);
  }
}