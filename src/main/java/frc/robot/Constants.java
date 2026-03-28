// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class TurretConstants {
    public static final double kTurretRatio = 10.0; //100T driven by 10T //FIXME

    //FIXME
    public static final double Front = 0.0;
    public static final double Back = 10.0;
    public static final double Left = 15.0;
    public static final double Right = 5.0;
    public static final double AutoDepot = 17.0;
    public static final double AutoOutpost = 13.0;

    public static final double CruiseVelocity = 3.5; //1 rotations per second cruise
    public static final double Acceleration = 8; //10 Take approx 0.5 seconds to reach max vel
    public static final double Jerk = 100; //Take approximately 0.1 seconds to reach max accel

    public static final double kP = 5;  //5 An error of 1 rotation results in kP V output
    public static final double kI = 0.05; //0 No output for integrated error
    public static final double kD = 0.15; //0.1 A velocity of 1 rps results in kD V output
    public static final double kV = 0.25; //0.12 A velocity target of 1 rps results in kV V output
    public static final double kS = 0.25; //0.25 Approximately kS V to get the mechanism moving, output to overcome static friction

    public static final boolean StatorEnable = true;
    public static final double StatorLimit = 60;

    public static final boolean SupplyEnable = true;
    public static final double SupplyLimit = 60;

    public static final double desiredRotations = 10;

    public static final double tempSpeed = 0.15; //FIXME
  }

 public static class SpeedConstants{
    public static final double kShooterSpeed = 0.60; //manual speed
    public static final double kTargetVelocity = 8; //target velocity for motion magic in rps
    public static final double kMaxSpeed = 20; //in rps (?)
    public static final double kAutoVelocity = 14.5;
    public static final double kTrenchVelocity = 36;

    //Intake
    public static final double kRollerSpeed = 0.85;
    public static final double kPivotSpeed = 0.2;
    //Spindexer
    public static final double kSpindexerSpeed = 0.40;
    public static final double kTransitionSpeed = .85;
  }

  public static class MotorConstants {
    public static final String CANbus = "CANivore26"; //Only swerve on Canivore.  Rest on RoboRio.
    public static final String rio = "rio";

    public static final int kShooterTopMotor = 11;
    public static final int kShooterBottomMotor = 12;
    public static final int kTurretMotor = 13;
    public static final int kTransitionMotor = 14;
    public static final int kSpindexerMotor = 15;
    public static final int kIntakePivotMotor = 16;
    public static final int kIntakeRollerMotor = 17;
  }
  

   public static class PositionConstants {
    public static final double kIntakeDown = 0; //FIXME
    public static final double kIntakeUp = -6.7; //FIXME
  }

}
