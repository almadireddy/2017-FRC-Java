package org.usfirst.frc.team4192.utilities;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by szydlikr on 3/4/2017.
 */
public class CollisionDetector {
  private AHRS ahrs;
  private double last_world_linear_accel_x;
  private double last_world_linear_accel_y;
  private boolean collisionDetected = false;
  
  private final static double kCollisionThreshold_DeltaG = 0.1f;
  
  public CollisionDetector(AHRS ahrs) {
    this.ahrs = ahrs;
  }
  
  private Thread collisionDetectionThread = new Thread(() -> {
    while (!Thread.interrupted()) {
      collisionDetected = false;
  
      double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
      double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
      last_world_linear_accel_x = curr_world_linear_accel_x;
      double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
      double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
      last_world_linear_accel_y = curr_world_linear_accel_y;
  
      if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
          ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
        collisionDetected = true;
      }
      SmartDashboard.putBoolean("CollisionDetected", collisionDetected);
      SmartDashboard.putNumber("jerkX", currentJerkX);
      SmartDashboard.putNumber("jerkY", currentJerkY);
    }
  });
  
  public void start() {
    collisionDetectionThread.start();
  }
  
  public boolean isCollisionDetected() {
    return collisionDetected;
  }
}
