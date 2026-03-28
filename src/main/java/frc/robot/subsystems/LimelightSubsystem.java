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
  private final double shooterHeight = 29.75;
  private final double hubHeight = 72.0;
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
    return Math.sqrt(Math.pow(hy, 2) - Math.pow(hi, 2));
  }

  public double getDistance(double distToCam, double angle) {
    angle = Math.abs(angle);
    double targetHeight = inToMeters(23.5); // meters

    return Math.sqrt(Math.pow(distToCam, 2) + Math.pow(targetHeight, 2) + 2 * targetHeight * distToCam * Math.cos(Math.toRadians(angle)));
  }

  public double getVelo(double dis, double drag) {
    double denom = 2 * Math.pow(Math.cos(Math.toRadians(62)), 2) * (dis * Math.tan(Math.toRadians(62)) - (inToMeters(hubHeight - shooterHeight)));
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
    return Math.asin((Math.sin((Math.toRadians(tx))) / dis) * inToMeters(23.5));
  }


  public double getTagYaw(int id)
  {
    LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{id});

    Pose3d tagPose = LimelightHelpers.getTargetPose3d_CameraSpace(name);
    return Units.radiansToDegrees(tagPose.getRotation().getY());
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

  /*public double getTurretSpeed()
  {
    double vel = -67;
    if(rawfids != null && rawfids.length != 0)
    {
      vel = .2;
      for(int id: vID)
      {
        for(RawFiducial fid : rawfids)
        {
          if(fid.id == id)
          {
            double yaw = fid.txnc;
            double offset = Math.toDegrees(getAngletoCenter(Math.abs(getTagYaw(fid.id) - fid.txnc), getDistance(getXLeg(fid.distToCamera, hubHeight - shooterHeight), getTagYaw(fid.id) - fid.txnc)));
            System.out.println("angle: " + Math.abs(getTagYaw(fid.id)));
            System.out.println("distance true: " + getDistance(getXLeg(fid.distToCamera, hubHeight - shooterHeight), getTagYaw(fid.id) - fid.txnc));
            if(yaw > 0) offset *= -1;
            if(yaw + offset > 3) vel *= -1;
            else if(yaw + offset < 3) vel = 0;
          }
        } 
      }
    }
    turHist.addFirst(vel);
    if(turHist.size() > 5) turHist.removeLast();
    for(double oldvel : turHist)
    {
      if(oldvel != -67) return oldvel;
    }
    return 0.0;
  }

  public void clearTurHist()
  {
    turHist.clear();
  }*/

  @Override
  public void periodic() {
    updateVision();
  }

}