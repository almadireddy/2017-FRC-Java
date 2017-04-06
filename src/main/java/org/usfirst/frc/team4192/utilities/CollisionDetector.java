package org.usfirst.frc.team4192.utilities;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by Al on 3/4/2017.
 */
public class CollisionDetector {
  private AHRS ahrs;
  private double lastWorldAccelX;
  private double lastWorldAccelY;
  private boolean collisionDetected = false;
  
  private final static double collisionThreshold = 0.5f;
  
  public CollisionDetector(AHRS ahrs) {
    this.ahrs = ahrs;
  }
  
  private Thread collisionDetectionThread = new Thread(() -> {
    while (!Thread.interrupted()) {
      collisionDetected = false;
  
      double currentWorldAccelX = ahrs.getWorldLinearAccelX();
      double currentJerkX = currentWorldAccelX - lastWorldAccelX;
      lastWorldAccelX = currentWorldAccelX;
      double currWorldAccelY = ahrs.getWorldLinearAccelY();
      double currentJerkY = currWorldAccelY - lastWorldAccelY;
      lastWorldAccelY = currWorldAccelY;
  
      if (Math.abs(currentJerkX) > collisionThreshold ||
          Math.abs(currentJerkY) > collisionThreshold) {
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
