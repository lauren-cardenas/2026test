// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.SpeedConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final TalonFX shooterTopMotor = new TalonFX(MotorConstants.kShooterTopMotor);
  private final TalonFX shooterBottomMotor = new TalonFX(MotorConstants.kShooterBottomMotor);

  private final DutyCycleOut talonOut = new DutyCycleOut(0);

  private final MotionMagicVelocityVoltage m_motionMagic = new MotionMagicVelocityVoltage(0).withSlot(0);

  private double lastSetpoint = 0.0;
  private double setPoint = 0.0;
  private double allowableError = 3.0;

  public ShooterSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    TalonFXConfiguration config2 = new TalonFXConfiguration();

    config.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    //set slot 0 gains
    var slot0Configs = config.Slot0;
    slot0Configs.kS = 0.25; //Add kS V output to overcome static friction, default 0.25
    slot0Configs.kV = 0.26; //A velocity target of 1 rps results in kV V output, default 0.12
    slot0Configs.kA = 0.54; //An acceleration of 1 rps/s requires kA V output, default 0.01
    slot0Configs.kI = 0; //no output for integrated error
    slot0Configs.kD = 0; //no output for error derivative

    var mm = config.MotionMagic;
    mm.MotionMagicAcceleration = 400; //Target acceleration of 400 rps/s (0.25 seconds to max)
    mm.MotionMagicJerk = 4000; //Target jerk of 4000 rps/s/s (0.1 seconds)

    config.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

    config2.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    //set slot 0 gains
    var slot0Configs2 = config.Slot0;
    slot0Configs2.kS = 0.25; //Add kS V output to overcome static friction, default 0.25
    slot0Configs2.kV = 0.26; //A velocity target of 1 rps results in kV V output, default 0.12
    slot0Configs2.kA = 0.54; //An acceleration of 1 rps/s requires kA V output, default 0.01
    slot0Configs2.kI = 0; //no output for integrated error
    slot0Configs2.kD = 0; //no output for error derivative

    var mm2 = config2.MotionMagic;
    mm2.MotionMagicAcceleration = 400; //Target acceleration of 400 rps/s (0.25 seconds to max)
    mm2.MotionMagicJerk = 4000; //Target jerk of 4000 rps/s/s (0.1 seconds)

    config.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

    shooterTopMotor.getConfigurator().apply(config);
    shooterBottomMotor.getConfigurator().apply(config);
    
    //shooterBottomMotor.setControl(new Follower(shooterTopMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Launcher Top RPM", shooterTopMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Launcher Bottom RPM", shooterBottomMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Launcher SetPoint", setPoint);
    SmartDashboard.putBoolean("Launcher Ready", isAtVelocity());

  }

  public void motionMagicSetVelocity(){
    //set velocity to 8 rps, add 0.5 V to overcome gravity
    shooterTopMotor.setControl(m_motionMagic.withVelocity(lastSetpoint));
    shooterBottomMotor.setControl(m_motionMagic.withVelocity(lastSetpoint));

  }

  public double getTopVelocity(){
    double velocity = shooterTopMotor.getVelocity().getValueAsDouble();
    return velocity;
  }

  public double getBottomVelocity(){
    double velocity = shooterBottomMotor.getVelocity().getValueAsDouble();
    return velocity;
  }
  
  public void setVelocitySetpoint(double velocity){
    lastSetpoint = velocity;
  }

  public void setVelocity(double velocity){
    setVelocitySetpoint(velocity); //velocity constants
  }

  public void launcherMaxSpeed(){
    lastSetpoint = setPoint;
    setPoint = SpeedConstants.kMaxSpeed;
  }

  public void setSetPoint(double point){
    lastSetpoint = setPoint;
    this.setPoint = point;
  }

  public boolean isAtVelocity(){
    double error = shooterTopMotor.getVelocity().getValueAsDouble() - setPoint;
    return (Math.abs(error) < allowableError);
  }

  public void stopLauncher(){
    lastSetpoint = setPoint;
    setPoint = 0;
  }

  public void manualSpeed(double speed){
    talonOut.Output = speed;
    talonOut.EnableFOC = true;
    shooterTopMotor.setControl(talonOut);
    shooterBottomMotor.setControl(talonOut);

  }
}
