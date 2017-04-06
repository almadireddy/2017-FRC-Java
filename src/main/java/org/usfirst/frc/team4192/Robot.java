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
  public static   CANTalon leftMaster;
  public static   CANTalon rightMaster;
  public static   CANTalon actuator;
  private static  CANTalon leftSlave;
  private static  CANTalon rightSlave;
  private static  CANTalon flywheel;
  private static  CANTalon lift;
  private static  CANTalon agitator;
  private static  VictorSP intake;
  private static  Spark trigger;
  
  public  static  JankoDrive jankoDrive;
  public  static  DigitalInput backSwitch;
  public  static  PIDController turnController;
  private static  AHRS ahrs;
  private         JaggernautJoystick joystick;
  private         NetworkTable stateTable;
  private         CollisionDetector collisionDetector;
  
  private static double flywheelTargetRPM;
  private static double driveSensitivity;
  private boolean scoringPosition = false;
  
  private void updateDriveConstants() {
    jankoDrive.setPID(
        JankoConstants.defaultDriveKp,
        JankoConstants.defaultDriveKi,
        JankoConstants.defaultDriveKd);
  }
  
  private void updateGyroConstants() {
    turnController.setPID(
        JankoConstants.defaultGyroKp,
        JankoConstants.defaultGyroKi,
        JankoConstants.defaultGyroKd);
  }
  
  private void updateFlywheelConstants() {
    flywheelTargetRPM = SmartDashboard.getNumber("flywheelTarget", 0.0);
    
    flywheel.setPID(
        JankoConstants.defaultFlywheelKp,
        JankoConstants.defaultFlywheelKi,
        JankoConstants.defaultFlywheelKd);
    
    flywheel.setSetpoint(flywheelTargetRPM);
  }
  
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
    intake        = new VictorSP(JankoConstants.intakeID);
    trigger       = new Spark(JankoConstants.triggerID);
    agitator      = new CANTalon(JankoConstants.agitatorID);
    lift          = new CANTalon(JankoConstants.liftID);
    rightMaster   = new CANTalon(JankoConstants.rightMasterID);
    rightSlave    = new CANTalon(JankoConstants.rightSlaveID);
    actuator      = new CANTalon(JankoConstants.actuatorID);
    flywheel      = new CANTalon(JankoConstants.flywheelID);
    leftSlave     = new CANTalon(JankoConstants.leftSlaveID);
    leftMaster    = new CANTalon(JankoConstants.leftMasterID);
  
    backSwitch    = new DigitalInput(0);
    jankoDrive    = new JankoDrive(leftMaster, leftSlave, rightMaster, rightSlave);
    jankoDrive.setSlewRate(60);
    driveSensitivity = 0.85;
  
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
  
    NetworkTable.setServerMode();
    stateTable = NetworkTable.getTable("stateTable");
    
    ahrs = new AHRS(SPI.Port.kMXP);
    DriverStation.reportWarning("instantiated navX MXP:  ", false);
    SmartDashboard.putBoolean("gyroPIDExists", true);
    
    turnController = new PIDController(
        0.01,
        0.0,
        0,
        0.0,
        ahrs,
        jankoDrive);
    turnController.setInputRange(-180.0, 180.0);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(3.0);
    turnController.setContinuous(true);
    turnController.disable();
    
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
        SmartDashboard.putNumber("actualHeading",
            ahrs.getAngle());
        
        SmartDashboard.putNumber("Climber Current",
            lift.getOutputCurrent());
        
        SmartDashboard.putNumber("flywheelMeasuredRPM",
            -flywheel.getEncVelocity());
        
        SmartDashboard.putNumber("targetRPM",
            flywheelTargetRPM);
        
        SmartDashboard.putNumber("Left Encoder Value",
            -jankoDrive.getLeftValue()/4096);
        
        SmartDashboard.putNumber("Right Encoder Value",
            jankoDrive.getRightValue()/4096);
        
        SmartDashboard.putBoolean("Back Limit Switch",
            backSwitch.get());
        
        SmartDashboard.putNumber("Actuator Encoder Position",
            actuator.getEncPosition());
      }
    });
    dashboardUpdateThread.start();
  
    collisionDetector.start();
  
    stateTable.putBoolean("autonomousMode", false);
    stateTable.putBoolean("teleop", false);
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
    stateTable.putBoolean("teleop", false);
    
    LeftAuton leftAuton;
    MiddleAuton middleAuton;
    RightAuton rightAuton;
    DefaultAuton defaultAuton;
    
    switch (SmartDashboard.getString("Selected Autonomous", "default")) {
      case "Left":
        leftAuton = new LeftAuton();
        leftAuton.start();
        break;
        
      case "Middle":
        middleAuton = new MiddleAuton();
        middleAuton.start();
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
  
  public static boolean gyroOnTarget() {
    return turnController.onTarget();
  }
  
  
  private void sensitivityControl() {
    if (joystick.buttonPressed(JankoConstants.sensitivityButton)) {
      if (driveSensitivity == 0.65)
        driveSensitivity = 0.90;
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
    jankoDrive.arcadeDrive(
        -joystick.getYaxis()*driveSensitivity,
        -joystick.getXaxis()*driveSensitivity,
        true);
  }
  
  private void intakeControl() {
    if (joystick.getLeftTrigger() > 0.5)
      intake.set(0.75);
    else
      intake.set(0);
    
    if (joystick.isHeldDown(JankoConstants.intakeReverse))
      intake.set(-0.75);
  }
  
  private void liftControl() {
    if (joystick.isHeldDown(JankoConstants.liftButton))
      lift.set(1);
    else
      lift.set(0);
  }
  
  private void flywheelControl() {
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
    updatePIDConstants();
    actuator.enable();
    
    if (turnController.isEnabled())
      turnController.disable();
    
    jankoDrive.prepareForTeleop();
    stateTable.putBoolean("autonomousMode", false);
    stateTable.putBoolean("teleop", true);
    
    jankoDrive.setSlewRate(60);
  }
  
  @Override
  public void teleopPeriodic() {
    joystick.update();
    driveControl();
    intakeControl();
    liftControl();
    flywheelControl();
    triggerControl();
    sensitivityControl();
    collisionRumble();
    actuatorControl();
  }
}