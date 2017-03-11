package org.usfirst.frc.team4192;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4192.autonRoutines.DefaultAuton;
import org.usfirst.frc.team4192.autonRoutines.LeftAuton;
import org.usfirst.frc.team4192.autonRoutines.MiddleAuton;
import org.usfirst.frc.team4192.autonRoutines.RightAuton;
import org.usfirst.frc.team4192.utilities.CollisionDetector;
import org.usfirst.frc.team4192.utilities.JaggernautJoystick;

/**
 * Created by Al on 1/22/2017.
 */
public class Robot extends IterativeRobot {
  public static CANTalon leftMaster;
  public static CANTalon rightMaster;
  private static CANTalon leftSlave;
  private static CANTalon rightSlave;
  public static CANTalon flywheel;
  private static CANTalon lift;
  public static CANTalon trigger;
  public static CANTalon agitator;
  
  public static VictorSP intake;
  
  public static JankoDrive jankoDrive;
  public static double driveSensitivity;
  
  private static int drivePIDThreshold = 10;
  
  private JaggernautJoystick joystick;
  private static AHRS ahrs;          // the NavX board, I'm calling it ahrs because thats what all the examples call it.
  
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
  
  
  private LeftAuton leftAuton;
  private MiddleAuton redMiddleAuton;
  private RightAuton rightAuton;
  
  private DefaultAuton defaultAuton;
  
  private NetworkTable stateTable;
  
  private CollisionDetector collisionDetector;
  
  //Flywheel Values
  private Boolean flywheelEnabled = false;
  
  ////// End Instance Variables //////
  
  // updates all the drive pid constants to what they are on the dashboard
  private void updateDriveConstants() {
    driveKp = SmartDashboard.getNumber("driveP", JankoConstants.defaultDriveKp);
    driveKi = SmartDashboard.getNumber("driveI", JankoConstants.defaultDriveKi);
    drivekd = SmartDashboard.getNumber("driveD", JankoConstants.defaultDriveKd);
    jankoDrive.setPID(0.05, 0, 0.0);
  }
  
  // updates all the gyro pid constants to what they are on the dashboard
  private void updateGyroConstants() {
    gyroKp = SmartDashboard.getNumber("gyroP", JankoConstants.defaultGyroKp);
    gyroKi = SmartDashboard.getNumber("gyroI", JankoConstants.defaultGyroKi);
    gyroKd = SmartDashboard.getNumber("gyroD", JankoConstants.defaultGyroKd);
    turnController.setPID(0.008, 0, 0);
  }
  
  // updates all the flywheelID pid constants to what they are on the dashboard
  private void updateFlywheelConstants() {
    flywheelKp = SmartDashboard.getNumber("flywheelP", JankoConstants.defaultFlywheelKp);
    flywheelKi = SmartDashboard.getNumber("flywheelI", JankoConstants.defaultFlywheelKi);
    flywheelKd = SmartDashboard.getNumber("flywheelD", JankoConstants.defaultFlywheelKd);
    flywheelKf = SmartDashboard.getNumber("flywheelF", JankoConstants.defaultFlywheelKf);
    flywheelTargetRPM = SmartDashboard.getNumber("flywheelTarget", 0.0);
    flywheel.setP(0.04);
    flywheel.setI(flywheelKi);
    flywheel.setD(0.7);
    flywheel.setF(0.0);
    flywheel.setSetpoint(flywheelTargetRPM);
  }
  
  // calls the three constants update functions
  private void updatePIDConstants() {
    updateDriveConstants();
    updateGyroConstants();
    updateFlywheelConstants();
  }
  
  public static void zeroSensors() {
    ahrs.reset();
  }
  
  @Override
  public void robotInit() {
    intake = new VictorSP(JankoConstants.intakeID);
    agitator = new CANTalon(JankoConstants.agitatorID);
    lift = new CANTalon(JankoConstants.liftID);
    rightMaster = new CANTalon(JankoConstants.rightMasterID);
    rightSlave = new CANTalon(JankoConstants.rightSlaveID);
    trigger = new CANTalon(JankoConstants.triggerID);
    flywheel = new CANTalon(JankoConstants.flywheelID);
    leftSlave = new CANTalon(JankoConstants.leftSlaveID);
    leftMaster = new CANTalon(JankoConstants.leftMasterID);
    
    NetworkTable.setServerMode();
    stateTable = NetworkTable.getTable("stateTable");
    
    jankoDrive = new JankoDrive(leftMaster, leftSlave, rightMaster, rightSlave);
    jankoDrive.setSlewRate(60);
    driveSensitivity = 0.8;
    
    flywheel.changeControlMode(CANTalon.TalonControlMode.Speed);
    flywheel.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
    flywheel.reverseSensor(true);
    flywheel.setProfile(0);
    flywheel.disable();
    
    lift.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    agitator.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    trigger.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    
    joystick = new JaggernautJoystick(JankoConstants.joystick);
    
    ahrs = new AHRS(SPI.Port.kMXP); // set the NavX board to use the MXP port in the middle of the roboRIO
    DriverStation.reportWarning("instantiated navX MXP:  ", false);
    SmartDashboard.putBoolean("gyroPIDExists", true);
    
    turnController = new PIDController(0.01, 0.0, 0, 0.0, ahrs, jankoDrive);
    turnController.setInputRange(-180.0, 180.0);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(3.0);
    turnController.setContinuous(true);
    turnController.disable();

    updatePIDConstants();
    
    collisionDetector = new CollisionDetector(ahrs);

    Thread flywheelControlThread = new Thread(() -> {
      while (!Thread.interrupted()) {
        updateFlywheelConstants();
        if (Math.abs(flywheel.getEncVelocity()) > 7500)
          SmartDashboard.putBoolean("flywheel ready", true);
        else
          SmartDashboard.putBoolean("flywheel ready", false);
      }
    });
    flywheelControlThread.start();
    
    Thread dashboardUpdateThread = new Thread(() -> {
      while (!Thread.interrupted()) {
        SmartDashboard.putNumber("actualHeading", ahrs.getAngle());
        SmartDashboard.putNumber("flywheelMeasuredRPM", -flywheel.getEncVelocity());
        SmartDashboard.putNumber("targetRPM", flywheelTargetRPM);
        SmartDashboard.putNumber("Left Encoder Value", jankoDrive.getLeftValue()/4096);
        SmartDashboard.putNumber("Right Encoder Value", -jankoDrive.getRightValue()/4096);
      }
    });
    dashboardUpdateThread.start();
  
    collisionDetector.start();
    CameraServer.getInstance().startAutomaticCapture();
  }
  
  @Override
  public void disabledInit() {
    stateTable.putBoolean("autonomousMode", false);
  }
  
  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
  }
  
  @Override
  public void autonomousInit() {
    zeroSensors();
    updatePIDConstants();
    stateTable.putBoolean("autonomousMode", true);
    switch (SmartDashboard.getString("Selected Autonomous", "default")) {
      case "Left":
        leftAuton = new LeftAuton();
        leftAuton.start();
        break;
        
      case "Middle":
        redMiddleAuton = new MiddleAuton();
        redMiddleAuton.start();
        break;
      
      case "Right":
        rightAuton = new RightAuton();
        rightAuton.start();
        break;
    
      default:
        defaultAuton = new DefaultAuton();
        defaultAuton.start();
        break;
    }
  }
  
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }
  
  
  private void sensitivityControl() {
    if (joystick.isHeldDown(JankoConstants.sensitivityButton)) driveSensitivity = 0.5;
    else driveSensitivity = 0.9;
  }
  
  private void triggerControl() {
    if (joystick.getRightTrigger() > 0) {
      trigger.set(-joystick.getRightTrigger() * 0.5);
      agitator.set(-1);
    }
    else {
      trigger.set(0);
      agitator.set(0);
    }
  }
  
  private void driveControl() {
    jankoDrive.arcadeDrive(-joystick.getYaxis()*driveSensitivity, -joystick.getXaxis()*driveSensitivity, true);
  }
  
  private void intakeControl() {
    if (joystick.getLeftTrigger() > 0.5)
      intake.set(0.75);
    else
      intake.set(0);
    if (joystick.isHeldDown(JankoConstants.intakeReverse)) {
      intake.set(-0.75);
    }
  }
  
  private void liftControl() {
    if (joystick.isHeldDown(JankoConstants.liftButton))
      lift.set(1);
    else
      lift.set(0);
  }
  
  private void flywheelPIDControl() {
    if (joystick.buttonPressed(JankoConstants.flywheelOn))
      if (flywheel.isEnabled()) {
        flywheel.disable();
      }
      else {
        flywheel.enable();
      }
  }
  
  private void flywheelBangBangControl() {
    if (joystick.buttonPressed(JankoConstants.flywheelOn)) {
      if (flywheel.get() > 0) {
        flywheelEnabled = !flywheelEnabled;
      }
      else {
        flywheelEnabled = true;
      }
    }
    
    if (flywheelEnabled) {
      if (Math.abs(flywheel.getEncVelocity()) > flywheelTargetRPM) {
        flywheel.set(0.2);
      }
      else
        flywheel.set(0.8);
    }
    else
      flywheel.set(0);
  }
  
  private void agitatorControl() {
    if (joystick.buttonPressed(JankoConstants.agitatorToggle)) {
      if (agitator.isEnabled()) {
        agitator.set(0);
        agitator.disable();
      }
      else {
        agitator.enable();
        agitator.set(-1);
      }
    }
  }
  
  private void collisionRumble() {
    if (collisionDetector.isCollisionDetected())
      joystick.rumble();
  }
  
  @Override
  public void teleopInit() {
    zeroSensors();
    
    if (turnController.isEnabled())
      turnController.disable();
    
    jankoDrive.prepareForTeleop();
    stateTable.putBoolean("autonomousMode", false);
  }
  
  @Override
  public void teleopPeriodic() {
    joystick.update();
    driveControl();
    intakeControl();
    liftControl();
    flywheelPIDControl();
//    agitatorControl();
    triggerControl();
    sensitivityControl();
    collisionRumble();
  }
  
  public static boolean gyroOnTarget() {
    return turnController.onTarget();
  }
}