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
  public static CANTalon actuator;
  public static CANTalon agitator;
  
  public static VictorSP intake;
  public static Spark trigger;
  
  public static JankoDrive jankoDrive;
  public static double driveSensitivity;
  
  public static DigitalInput backSwitch;
  public static DigitalInput frontSwitch;
  
  private static int drivePIDThreshold = 10;
  
  boolean scoringPosition = false;
  
  private JaggernautJoystick joystick;
  private static AHRS ahrs;          // the NavX board, I'm calling it ahrs because thats what all the examples call it.
  
  private static double driveKp = 0.04;     // Drive PID constants
  private static double driveKi = 0.0;
  private static double driveKd = 0.25;
  
  private static double gyroKp = 0.08;      // Gyroscope PID constants
  private static double gyroKi = 0;
  private static double gyroKd = 0;
  
  private static double flywheelKp = 0.04;  // Drive PID constants
  private static double flywheelKi = 0.0;
  private static double flywheelKd = 0.7;
  private static double flywheelTargetRPM;
  
  public static PIDController turnController;
  
  private LeftAuton leftAuton;
  private MiddleAuton redMiddleAuton;
  private RightAuton rightAuton;
  
  private DefaultAuton defaultAuton;
  
  private NetworkTable stateTable;
  
  private CollisionDetector collisionDetector;
  
  //Flywheel Values
  private boolean flywheelEnabled = false;
  
  ////// End Instance Variables //////
  
  // updates all the drive pid constants to what they are on the dashboard
  private void updateDriveConstants() {
    driveKp = SmartDashboard.getNumber("driveP", JankoConstants.defaultDriveKp);
    driveKi = SmartDashboard.getNumber("driveI", JankoConstants.defaultDriveKi);
    driveKd = SmartDashboard.getNumber("driveD", JankoConstants.defaultDriveKd);
    jankoDrive.setPID(driveKp, driveKi, driveKd);
  }
  
  
  // updates all the gyro pid constants to what they are on the dashboard
  private void updateGyroConstants() {
    gyroKp = SmartDashboard.getNumber("gyroP", JankoConstants.defaultGyroKp);
    gyroKi = SmartDashboard.getNumber("gyroI", JankoConstants.defaultGyroKi);
    gyroKd = SmartDashboard.getNumber("gyroD", JankoConstants.defaultGyroKd);
    turnController.setPID(gyroKp, gyroKi, gyroKd);
  }
  
  // updates all the flywheelID pid constants to what they are on the dashboard
  private void updateFlywheelConstants() {
    flywheelKp = SmartDashboard.getNumber("flywheelP", JankoConstants.defaultFlywheelKp);
    flywheelKi = SmartDashboard.getNumber("flywheelI", JankoConstants.defaultFlywheelKi);
    flywheelKd = SmartDashboard.getNumber("flywheelD", JankoConstants.defaultFlywheelKd);
    flywheelTargetRPM = SmartDashboard.getNumber("flywheelTarget", 0.0);
    flywheel.setPID(flywheelKp, flywheelKi, flywheelKd);
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
    trigger = new Spark(JankoConstants.triggerID);
    agitator = new CANTalon(JankoConstants.agitatorID);
    lift = new CANTalon(JankoConstants.liftID);
    rightMaster = new CANTalon(JankoConstants.rightMasterID);
    rightSlave = new CANTalon(JankoConstants.rightSlaveID);
    actuator = new CANTalon(JankoConstants.actuatorID);
    flywheel = new CANTalon(JankoConstants.flywheelID);
    leftSlave = new CANTalon(JankoConstants.leftSlaveID);
    leftMaster = new CANTalon(JankoConstants.leftMasterID);
  
    backSwitch = new DigitalInput(0);
    frontSwitch = new DigitalInput(1);
    
    NetworkTable.setServerMode();
    stateTable = NetworkTable.getTable("stateTable");
    
    jankoDrive = new JankoDrive(leftMaster, leftSlave, rightMaster, rightSlave);
    jankoDrive.setSlewRate(60);
    driveSensitivity = 1.0;
    
    flywheel.changeControlMode(CANTalon.TalonControlMode.Speed);
    flywheel.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
    flywheel.reverseSensor(true);
    flywheel.setProfile(0);
    flywheel.disable();
    
    lift.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    agitator.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    actuator.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
    actuator.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
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
        SmartDashboard.putNumber("Left Encoder Value", -jankoDrive.getLeftValue()/4096);
        SmartDashboard.putNumber("Right Encoder Value", jankoDrive.getRightValue()/4096);
        SmartDashboard.putBoolean("Front Limit Switch", frontSwitch.get());
        SmartDashboard.putBoolean("Back Limit Switch", backSwitch.get());
        SmartDashboard.putNumber("Actuator Encoder Position", actuator.getEncPosition());
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
    if (joystick.buttonPressed(JankoConstants.sensitivityButton)) {
      if (driveSensitivity == 0.65)
        driveSensitivity = 1;
      else
        driveSensitivity = 0.65;
    }
  }
  
  private void triggerControl() {
    if (joystick.getRightTrigger() > 0) {
      trigger.set(-joystick.getRightTrigger() * 0.5 + 0.15);
      agitator.set(-1);
    }
    else {
      trigger.set(0);
      agitator.set(0);
    }
  }
  
  private void actuatorControl() {
    if (joystick.buttonPressed(JankoConstants.actuatorToggle)) {
      if (scoringPosition) {
        actuator.setSetpoint(JankoConstants.gearLoadPosition);
        scoringPosition = false;
      } else {
        actuator.setSetpoint(JankoConstants.gearScorePosition);
        scoringPosition = true;
      }
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
  
  int var = 0;
  
  @Override
  public void teleopPeriodic() {
    if (var == 0) {
      while (backSwitch.get()) {
        actuator.set(-0.15);
      }
      actuator.set(0);
      actuator.changeControlMode(CANTalon.TalonControlMode.Position);
      actuator.setEncPosition(0);
      
      var = 1;
    }
    joystick.update();
    driveControl();
//    intakeControl();
    liftControl();
    flywheelPIDControl();
    triggerControl();
    sensitivityControl();
    collisionRumble();
    actuatorControl();
  }
  
  public static boolean gyroOnTarget() {
    return turnController.onTarget();
  }
}