package frc.robot.subsystems;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.ArrayDeque;


import java.lang.Math;

public class LimelightSubsystem extends SubsystemBase {
  public String name = "limelight-berny";

  private RawFiducial[] rawfids;
  private final Set<Integer> vID = new LinkedHashSet<Integer>(Set.of(7, 26, 5, 24, 2, 18));
  private static final double shooterHeight = 29.75;
  private static final double hubHeight = 72.0;
  private static final double SHOOT_ANGLE = 62;
  private static final double TAG_OFFSET = 23.5;
  private static final double TURRET_DEADBAND = 3;
  private static final double ARB_NUMBER = -6;
  private ArrayDeque<Double> speedHist = new ArrayDeque<>();
  private ArrayDeque<Double> turHist = new ArrayDeque<>();

  public void updateVision() {
    rawfids = LimelightHelpers.getRawFiducials(name);
  }

  public boolean isInList(int[] list, int tar) {
    for (int id : list) {
      if (tar == id) {
        return true;
      }
    }
    return false;
  }

  public double getXLeg(double hy, double hi) {
  double val = (hy * hy) - (hi * hi);
  if (val <= 0 || Double.isNaN(val)) return 0.0;
  return Math.sqrt(val);
}

  public double getDistance(double distToCam, double angle) {
  if (Double.isNaN(distToCam) || Double.isInfinite(distToCam)) return 0.0;

  double targetHeight = inToMeters(TAG_OFFSET);
  angle = Math.toRadians(Math.abs(angle));

  double val = distToCam * distToCam +
  targetHeight * targetHeight +
  2 * targetHeight * distToCam * Math.cos(angle);

  if (val <= 0 || Double.isNaN(val)) return 0.0;

  return Math.sqrt(val);
  }

  public double getVelo(double dis, double drag) {
    double denom = 2 * Math.pow(Math.cos(Math.toRadians(SHOOT_ANGLE)), 2) * (dis * Math.tan(Math.toRadians(SHOOT_ANGLE)) - (inToMeters(hubHeight - shooterHeight)));
    if (denom <= 0) return 0;

    return drag * Math.sqrt((9.80667 * Math.pow(dis, 2)) / denom);
  }

  public double velocityToRps(double v, double r) {
    return (v / (2 * Math.PI * r));
  }


  public double inToMeters(double in) {
    return in * .0254;
  }

  public double getAngletoCenter(double tx, double dis) {
  double offset = inToMeters(TAG_OFFSET);

  if (Double.isNaN(dis) || Double.isInfinite(dis) || dis <= 1e-6) {
    return 0.0;
  }

  double val = (Math.sin(Math.toRadians(tx)) * offset) / dis;

  val = Math.max(-1.0, Math.min(1.0, val));

  double result = Math.asin(val);

  if (Double.isNaN(result) || Double.isInfinite(result)) {
    return 0.0;
  }

  return result;
}


  public double getTagYaw(int id) {

    Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace(name);
    if (tagPose == null) {
        return 0.0;
    }

    double yaw = Units.radiansToDegrees(tagPose.getRotation().getY());
    if (Double.isNaN(yaw) || Double.isInfinite(yaw)) {
        return 0.0;
    }

    return yaw;
}

  public double getSpeed() {
    double vel = 0.0;
    if (rawfids != null && rawfids.length != 0)
    {
      for (int id : vID) {
        for (RawFiducial fid : rawfids) {
          if (fid.id == id) {
            vel = velocityToRps(getVelo(getDistance(getXLeg(fid.distToCamera, inToMeters(hubHeight - shooterHeight)), Math.toRadians(getTagYaw(fid.id))), 1), inToMeters(1.5));
          }
        }
      }
    }
    speedHist.addFirst(vel);
    if(speedHist.size() > 5) speedHist.removeLast();
    for(double oldvel : speedHist)
    {
      if(oldvel != 0.0) return oldvel;
    }
    return 0.0;
  }

  public void clearSpeedHist()
  {
    speedHist.clear();
  }

  public double getTurretSpeed()
  {
    double vel = ARB_NUMBER; // this number is just a place holder telling it the velocity is not found from the april tags
    if(rawfids != null && rawfids.length != 0)
    {
      vel = .2;
      for(int id: vID)
      {
        for(RawFiducial fid : rawfids)
        {
          if(fid.id == id)
          { // iterate through all of the ids that are valid and in the vID list
            double yaw = fid.txnc;
            double offset = Math.toDegrees(getAngletoCenter(Math.abs(getTagYaw(fid.id) - fid.txnc), getDistance(getXLeg(fid.distToCamera, inToMeters(hubHeight - shooterHeight)), getTagYaw(fid.id) - fid.txnc)));
            if(yaw > 0) offset *= -1;
            if(yaw + offset > TURRET_DEADBAND) vel *= -1;
            else if(yaw + offset < TURRET_DEADBAND) vel = 0;
          }
        } 
      }
    }
    turHist.addFirst(vel);
    if(turHist.size() > 5) turHist.removeLast();
    for(double oldvel : turHist)
    {
      if(oldvel != ARB_NUMBER) return oldvel;
    }
    //maybe turn the turret 180 degrees when you dont see an april tag?
    return 0.0;
  }

  public void clearTurHist()
  {
    turHist.clear();
  }

  @Override
  public void periodic() {
    updateVision();
  }

}