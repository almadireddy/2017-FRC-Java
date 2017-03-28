package org.usfirst.frc.team4192;

/**
 * Created by Al on 2/24/2017.
 */

/*
 * This class contains all the port mappings on Janko Fett
 * TODO: Replace with actual port mappings
 */
public class JankoConstants {
  public static int intakeID        = 0;  // intake is on victor
  public static int triggerID       = 1;  // trigger is on spark
  public static int agitatorID      = 1;
  public static int liftID          = 2;
  public static int rightMasterID   = 3;
  public static int rightSlaveID    = 4;
  public static int actuatorID      = 5;
  public static int flywheelID      = 6;
  public static int leftSlaveID     = 7;
  public static int leftMasterID    = 8;
  
  public static int joystick        = 0;
  
  public static int intakeReverse     = 1;
  public static int flywheelOn        = 2;
  public static int sensitivityButton = 3;
  public static int actuatorToggle    = 4;
  public static int actuatorScore     = 8;
  public static int agitatorToggle    = 5;
  public static int liftButton        = 6;
  
  public static int intakeAxis        = 2;
  public static int triggerWheelAxis  = 3;
  
  public static int reverseLimitSwitch = 0;
  public static int forwardLimitSwitch = 1;
  
  public static double defaultDriveKp = 0.04;
  public static double defaultDriveKi = 0.0;
  public static double defaultDriveKd = 0.25;
  
  public static double defaultGyroKp = 0.008;
  public static double defaultGyroKi = 0.0;
  public static double defaultGyroKd = 0.0;
  
  public static double defaultFlywheelKp = 0.04;
  public static double defaultFlywheelKi = 0.0;
  public static double defaultFlywheelKd = 0.7;
  public static double defaultFlywheelKf = 0.2;
  
  public static double gearScorePosition = 65;
  public static double gearLoadPosition  = 5;
}