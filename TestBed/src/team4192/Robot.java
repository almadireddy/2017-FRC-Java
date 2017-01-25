package team4192;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by Al on 1/22/2017.
 */
public class Robot extends IterativeRobot {
  public TalonSRX frontLeft;
  public VictorSP frontRight;
  public VictorSP backLeft;
  public VictorSP backRight;
  
  private RobotDrive drive;
  private Joystick joystick;
  private ADXRS450_Gyro gyro;          // the NavX board, I'm calling it AHRS because thats what all the examples call it.
  
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
  
  
  // updates all the gyro pid constants to what they are on the dashboard
  private void updateGyroConstants() {
    gyroKp = Double.parseDouble(SmartDashboard.getData("gyroP").toString());
    gyroKi = Double.parseDouble(SmartDashboard.getData("gyroI").toString());
    gyroKd = Double.parseDouble(SmartDashboard.getData("gyroD").toString());
  }
  
  /**
   * @param angle the angle you want to turn to, from -180 to 180
   */
  private void rotateToAngle(double angle) {
    SmartDashboard.putNumber("targetHeading", angle);
    SmartDashboard.putNumber("actualHeading", gyro.getAngle());
    turnController.enable();
    turnController.setSetpoint(angle);
    if (turnController.onTarget())
      turnController.disable();
  }
  
  @Override
  public void robotInit() {
    frontLeft = new TalonSRX(0);                // make CAN Talon SRX objects
    frontRight = new VictorSP(1);
    backLeft = new VictorSP(2);
    backRight = new VictorSP(3);
    
    frontLeft.setInverted(true);   // These might not need to be inverted.
    frontRight.setInverted(true);
    backLeft.setInverted(true);
    backRight.setInverted(true);
    
    gyro = new ADXRS450_Gyro();
    
    drive = new RobotDrive(frontLeft, frontRight);
    drive.setExpiration(0.1);
    
    joystick = new Joystick(0);
    
    autonTimer = new Timer();
    
    gyroPID = new gyroPID(frontLeft, frontRight);
    turnController = new PIDController(gyroKp, gyroKi, gyroKd, gyro, gyroPID);
    turnController.setInputRange(-180.0f, 180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(gyroTolerance);
    turnController.setContinuous(true);
    turnController.disable();
      
    
    // Another thread that handles updating all the values on driverstation so that teleopPeriodic
    // doesn't get hung up in the case of slow network speeds.
    Thread dashboardUpdateThread = new Thread(() -> {
      while (!Thread.interrupted()) {
        SmartDashboard.putNumber("actualHeading", gyro.getAngle());
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
    gyro.reset();       // reset the gyro board
    
    autonTimer.reset();
    autonTimer.start();
  }
  
  @Override
  public void autonomousPeriodic() {
    
  }
  
  @Override
  public void teleopInit() {
  }
  
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("actualHeading", gyro.getAngle());
    SmartDashboard.putNumber("targetHeading", 90.0);
    drive.arcadeDrive(joystick.getY(), joystick.getX(), true);  // if the motors don't need to be inverted, add negatives to the axes.
  }
}