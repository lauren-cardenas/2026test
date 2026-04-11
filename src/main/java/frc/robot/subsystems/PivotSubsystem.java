// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */

  private final TalonFX m_pivotMotor = new TalonFX(MotorConstants.kIntakePivotMotor);

  private final DutyCycleOut m_pivotRequest = new DutyCycleOut(0);

  //private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(0);

  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  private double pivotSetpoint = 0.0;

  private double allowableError = 1.0;

  public PivotSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //FIXME
    //cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 70.9;
    //cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.StatorCurrentLimit = 40; //Output Current Limit
    //cfg.CurrentLimits.SupplyTimeThreshold = 5; //Amont of time to allow current over supply limit
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 60; //Supply Current Limit
    cfg.MotionMagic.MotionMagicCruiseVelocity = 50; //50; // 5 rotations per second cruise
    cfg.MotionMagic.MotionMagicAcceleration = 100; //100; // Take approximately 0.5 seconds to reach max vel
    cfg.MotionMagic.MotionMagicJerk = 1000; //1000;// Take approximately 0.2 seconds to reach max accel 
    cfg.TorqueCurrent.PeakForwardTorqueCurrent = 60; //Current Limit value used in FOC Torque Mode
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = 60; //Current Limit value used in FOC Torque Mode
    cfg.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //SmartDashboard.putNumber("Arm Position", m_pivotMotor.getRotorPosition().getValueAsDouble());
  }

  public void motionMagicSetPosition(){
    m_pivotMotor.setControl(m_mmReq.withPosition(pivotSetpoint).withSlot(0));
  }

  public void setZero(){
    m_pivotMotor.setPosition(0);
  }

  public double getPosition(){
    double position = m_pivotMotor.getPosition().getValueAsDouble();
    return position;
  }

  public void setPivotSetpoint(double position){
    pivotSetpoint = position / 24.0; //FIXME Pivot GearRatio
  }

  public void setPosition(double position){
    setPivotSetpoint(position); //position constants
  }

  public boolean isAtPosition(){
    double error = getPosition() - pivotSetpoint;
    return (Math.abs(error) < allowableError);
  }

  public void manualPivot(double speed){
    m_pivotRequest.Output = speed;
    m_pivotRequest.EnableFOC = true;
    m_pivotMotor.setControl(m_pivotRequest);
  }

}
