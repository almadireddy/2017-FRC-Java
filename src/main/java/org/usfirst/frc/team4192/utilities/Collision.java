package org.usfirst.frc.team4192.utilities;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Created by szydlikr on 3/4/2017.
 */
public class Collision {
  private AHRS ahrs;
  private double lastAccelX, lastAccelY;
  private double collisionThreshold = 0.5;
  boolean collisionDetected;
  double currentAccelX, currentJerkX, currentAccelY, currentJerkY;
  
  public Collision(AHRS ahrs) {
    this.ahrs = ahrs;
  }
  
  Thread collisionDetectionThread = new Thread(() -> {
    while (!Thread.interrupted()) {
      collisionDetected = false;
  
      currentAccelX = ahrs.getWorldLinearAccelX();
      currentJerkX = currentAccelX - lastAccelX;
      lastAccelX = currentAccelX;
  
      currentAccelY = ahrs.getWorldLinearAccelY();
      currentJerkY = currentAccelY - lastAccelY;
      lastAccelY = currentAccelY;
      
      if (Math.abs(currentJerkX) > collisionThreshold || Math.abs(currentJerkY) > collisionThreshold)
        collisionDetected = true;
  
      SmartDashboard.putBoolean("Collision Detected", true);
    }
  });
  
  public void start() {
    collisionDetectionThread.start();
  }
  
  public boolean isCollisionDetected() {
    return collisionDetected;
  }
}
