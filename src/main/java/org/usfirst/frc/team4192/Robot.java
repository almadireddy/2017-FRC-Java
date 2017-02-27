package org.usfirst.frc.team4192;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4192.autonRoutines.*;
import org.usfirst.frc.team4192.utilities.JaggernautGyroDrive;
import org.usfirst.frc.team4192.utilities.JaggernautJoystick;

/**
 * Created by Al on 1/22/2017.
 */
public class Robot extends IterativeRobot {
  public static CANTalon frontLeft;
  public static CANTalon frontRight;
  public static CANTalon rearLeft;
  public static CANTalon rearRight;
  public static CANTalon flywheel;
  public static CANTalon lift;
  public static CANTalon intake;
  public static CANTalon agitator;
  
  public JankoDrive jankoDrive;
  
  private static int drivePIDThreshold = 10;
  
  private JaggernautJoystick joystick;
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
  private JaggernautGyroDrive jaggernautGyroDrive;
    
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
    driveKp = SmartDashboard.getNumber("driveP", 0.0);
    driveKi = SmartDashboard.getNumber("driveI", 0.0);
    drivekd = SmartDashboard.getNumber("driveD", 0.0);
    setDriveConstants();
  }
  
  // updates all the gyro pid constants to what they are on the dashboard
  public static void updateGyroConstants() {
    gyroKp = SmartDashboard.getNumber("gyroP", 0.0);
    gyroKi = SmartDashboard.getNumber("gyroI", 0.0);
    gyroKd = SmartDashboard.getNumber("gyroD", 0.0);
    setGyroConstants();
  }
  
  // updates all the flywheelID pid constants to what they are on the dashboard
  public static void updateFlywheelConstants() {
    flywheelKp = SmartDashboard.getNumber("flywheelP", 0.0);
    flywheelKi = SmartDashboard.getNumber("flywheelI", 0.0);
    flywheelKd = SmartDashboard.getNumber("flywheelD", 0.0);
    flywheelKf = SmartDashboard.getNumber("flywheelF", 0.0);
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
    flywheelTargetRPM = SmartDashboard.getNumber("targetRPMControl", 0.0);
    setFlywheelTargetRPM();
  }
    
  public static boolean driveOnTarget() {
    return (frontLeft.getClosedLoopError() < drivePIDThreshold) && (frontRight.getClosedLoopError() < drivePIDThreshold);
  }

  public static boolean gyroOnTarget() {
    return turnController.onTarget();
  }
  
  private void zeroSensors() {
    ahrs.reset();
  }
  
  @Override
  public void robotInit() {
    frontLeft = new CANTalon(PortMap.frontLeftID);
    frontRight = new CANTalon(PortMap.frontRightID);
    rearLeft  = new CANTalon(PortMap.rearLeftID);
    rearRight = new CANTalon(PortMap.rearRightID);
    
    jankoDrive = new JankoDrive(frontLeft, rearLeft, frontRight, rearRight);
    jankoDrive.setExpiration(0.1);

    flywheel = new CANTalon(PortMap.flywheelID);
    lift = new CANTalon(PortMap.liftID);
    intake = new CANTalon(PortMap.intakeID);
    agitator = new CANTalon(PortMap.agitatorID);
    
    flywheel.changeControlMode(CANTalon.TalonControlMode.Speed);
    flywheel.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogEncoder);
    flywheel.setProfile(0);
    
    joystick = new JaggernautJoystick(PortMap.joystick);
    
    try {
      ahrs = new AHRS(SPI.Port.kMXP); // set the NavX board to use the MXP port in the middle of the roboRIO
      gyroExists = true;
      DriverStation.reportWarning("instantiated navX MXP:  ", false);
      SmartDashboard.putBoolean("gyroPIDExists", true);
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
      gyroExists = false;
      SmartDashboard.putBoolean("gyroPIDExists", false);
    }

    if (gyroExists) {
      jaggernautGyroDrive = new JaggernautGyroDrive(frontLeft, frontRight);
      turnController = new PIDController(0.01, 0.0, 0, ahrs, jaggernautGyroDrive);
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
  }
  
  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
  }
  
  @Override
  public void autonomousInit() {
    zeroSensors();
    jankoDrive.prepareForAuton();
    
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
  
  private void intakeControl() {
    if (joystick.isHeldDown(6))
      intake.set(1);
    else if (joystick.isHeldDown(7))
      intake.set(-1);
    else
      intake.set(0);
  }
  
  private void flywheelControl() {
    if (joystick.buttonPressed(PortMap.flywheelToggle)) {
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
  }
  
  @Override
  public void teleopPeriodic() {
    jankoDrive.arcadeDrive(joystick.getYaxis(), joystick.getXaxis(), true);
    joystick.update();
    intakeControl();
    flywheelControl();
  }
}