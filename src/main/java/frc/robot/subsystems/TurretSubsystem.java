// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  private final TalonFX talon = new TalonFX(MotorConstants.kTurretMotor);

  private final DutyCycleOut talonOut = new DutyCycleOut(0);

  private final MotionMagicVoltage m_motionMagic = new MotionMagicVoltage(0);

  private double turretSetpoint = 0.0;

  private double allowableError = 1.0;

  public TurretSubsystem() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    FeedbackConfigs fdb = config.Feedback;
    fdb.SensorToMechanismRatio = TurretConstants.kTurretRatio;

     /*Configure Motion Magic */
    MotionMagicConfigs mm = config.MotionMagic;
    mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(TurretConstants.CruiseVelocity))
      .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(TurretConstants.Acceleration))
      .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(TurretConstants.Jerk));

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 6.42;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -6.42;
    config.CurrentLimits.StatorCurrentLimitEnable = TurretConstants.StatorEnable;
    config.CurrentLimits.StatorCurrentLimit = TurretConstants.StatorLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = TurretConstants.SupplyEnable;
    config.CurrentLimits.SupplyCurrentLimit = TurretConstants.SupplyLimit;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 50;
    config.TorqueCurrent.PeakReverseTorqueCurrent = 50;
    config.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    m_motionMagic.EnableFOC = true;

    Slot0Configs Slot0 = config.Slot0;

    Slot0.kP = TurretConstants.kP; // An error of 1 rotation results in kP V output
    Slot0.kI = TurretConstants.kI; //No output for integrated error
    Slot0.kD = TurretConstants.kD; //A velocity of 1 rps results in kD V output
    Slot0.kV = TurretConstants.kV; //Velocity target of 1 rps results in kV V output
    Slot0.kS = TurretConstants.kS; //Approximately kS V to get the mechanism moving

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i=0; i < 5; ++i) {
      status = talon.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()){
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    talon.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Position", talon.getRotorPosition().getValueAsDouble());
  }

  public void setZero(){
    talon.setPosition(0.0);
  }

  public double getPosition(){
    double position = talon.getPosition().getValueAsDouble();
    return position;
  }

  public void manualDrive(double value){
    talonOut.Output = value;
    talonOut.EnableFOC = true;
    talon.setControl(talonOut);
  }

  public void setTurretSetpoint(double position){
    turretSetpoint = position / TurretConstants.kTurretRatio;
  }

  public void motionMagicSetPosition(){
    talon.setControl(m_motionMagic.withPosition(turretSetpoint).withSlot(0));
  }

  public boolean isAtHeight(){
    double error = getPosition() - turretSetpoint;
    return (Math.abs(error) < allowableError);
  }

  public void setElevator(double height){
    setTurretSetpoint(height); //turret constants
  }

  public void turnRight(double speed) //Limits were introduced earlier in the code, were good -David
  {
    talon.set(speed);
  }

  public void turnLeft(double speed)
  {
    talon.set(speed);
  }

  public void stop(){
    talon.set(0);
  }

}
