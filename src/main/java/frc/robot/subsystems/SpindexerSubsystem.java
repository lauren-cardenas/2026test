// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SpeedConstants;

public class SpindexerSubsystem extends SubsystemBase {
  /** Creates a new SpindexerSubsystem. */

  private final TalonFX m_transitionMotor = new TalonFX(MotorConstants.kTransitionMotor);
  private final TalonFX m_spindexerMotor = new TalonFX(MotorConstants.kSpindexerMotor);

  private final DutyCycleOut m_transitionRequest = new DutyCycleOut(0);
  private final DutyCycleOut m_spindexerRequest = new DutyCycleOut(0);

  public SpindexerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void fuelRun(){
    m_transitionRequest.Output = -SpeedConstants.kTransitionSpeed;
    m_transitionRequest.EnableFOC = true;
    m_transitionMotor.setControl(m_transitionRequest);

    m_spindexerRequest.Output = SpeedConstants.kSpindexerSpeed;
    m_spindexerRequest.EnableFOC = true;
    m_spindexerMotor.setControl(m_spindexerRequest);
  }

  public void stopFuel(){
    m_spindexerMotor.set(0);
    m_transitionMotor.set(0);
  }
}
