package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
import frc.robot.LimelightHelpers.RawFiducial;

public class LimelightSubsystem extends SubsystemBase {

  private final String name = "limelight-berny";

  private boolean tv;
  private RawFiducial[] rawfiducials;

  public LimelightSubsystem() {}

  /** Update tracking from Limelight safely */
  public void updateLimeLightTracking() {
    try {
      tv = LimelightHelpers.getTV(name);
      rawfiducials = LimelightHelpers.getRawFiducials(name);
    } catch (Exception e) {
      tv = false;
      rawfiducials = null;
      SmartDashboard.putString("Limelight Error", "Failed to read Limelight data");
    }
  }

  /** Get yaw of a specific tag */
  public double getTagYaw(int tagId) {
    if (rawfiducials == null) return -1;

    for (RawFiducial fid : rawfiducials) {
      if (fid.id == tagId) {
        try {
          double[] tagPose = NetworkTableInstance.getDefault()
              .getTable(name)
              .getEntry("botpose_targetspace")
              .getDoubleArray(new double[6]);
          if (tagPose.length >= 6) {
            return tagPose[5];
          }
        } catch (Exception e) {
          return -1; // Safe fallback
        }
      }
    }
    return -1;
  }

  /** Distance calculation */
  public double getDistance(double distToCam, double angle) {
    angle = Math.abs(angle);
    double targetHeight = 23.5 * 0.0254; // meters

    // Prevent negative or invalid math
    double inner = Math.pow(distToCam, 2) + Math.pow(targetHeight, 2) + 2 * targetHeight * distToCam * Math.cos(Math.toRadians(angle));
    if (inner < 0) return 0;

    return Math.sqrt(inner);
  }

  /** Shooter velocity calculation */
  public double getVelo(double dis, double drag) {
    double denom = 2 * Math.pow(Math.cos(Math.toRadians(28)), 2) * (dis * Math.tan(Math.toRadians(28)) - ((72 - 27) * 0.0254));
    if (denom <= 0) return 0;

    return drag * Math.sqrt((9.80667 * Math.pow(dis, 2)) / denom);
  }

  /** Angle to center calculation */
  public double getAngletoCenter(double tx, double dis) {
    tx = Math.abs(tx);
    double targetOffsetMeters = 23.5 * 0.0254;
    double ratio = targetOffsetMeters / dis;
    if (ratio > 1) ratio = 1; // Clamp to avoid asin domain error
    return Math.asin(Math.sin(Math.toRadians(tx)) * ratio);
  }

  /** Helper to check if tag is in list */
  public boolean isInList(int[] arr, int target) {
    if (arr == null) return false;
    for (int tar : arr) {
      if (target == tar) return true;
    }
    return false;
  }

  /** Returns shooter speed safely */
  public double getSpeedShooter() {
    if (!tv || rawfiducials == null) return 0;

    int[] valId = {2, 5, 4, 10};

    for (RawFiducial fid : rawfiducials) {
      if (isInList(valId, fid.id)) {
        double d = getDistance(fid.distToCamera, getTagYaw(fid.id));
        double vel = 0;
        if (d > 1.42) {
          vel = getVelo(d, 1.1);
        }
        return (vel / (Math.PI * (3 * 0.0254))) * 60;
      }
    }

    return 0;
  }

  /** Returns turret speed safely */
  public double getSpeedTurret(double turSpeed) {
    if (!tv || rawfiducials == null) return 0;

    int[] valId = {2, 5, 4, 10};

    for (RawFiducial fid : rawfiducials) {
      double change = 1;

      if (isInList(valId, fid.id)) {
        if (fid.txnc < 0) change = -1;

        double offset = getAngletoCenter(getTagYaw(fid.id), fid.distToCamera);

        if (fid.txnc + (offset * change) < 1) return -turSpeed;
        else if (fid.txnc + (offset * change) > 1) return turSpeed;
        else return 0;
      }
    }

    return 0;
  }

  /** Periodic update */
  @Override
  public void periodic() {
    updateLimeLightTracking();

    SmartDashboard.putBoolean("LL Has Target", tv);
    SmartDashboard.putNumber("LL Fiducial Count", rawfiducials == null ? 0 : rawfiducials.length);

    try {
      SmartDashboard.putNumber("Limelight Speed", getSpeedShooter());
    } catch (Exception e) {
      SmartDashboard.putString("Limelight Error", e.getMessage());
    }
  }
}