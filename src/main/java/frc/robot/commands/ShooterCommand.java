// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterCommand extends Command {
  /** Creates a new ShooterCommand. */
  ShooterSubsystem shooter;
  SpindexerSubsystem spinner;
  double velocity;
  boolean finishes;
  
  public ShooterCommand(double velocity, ShooterSubsystem shooter, SpindexerSubsystem spinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.spinner = spinner;
    this.velocity = velocity;
    this.finishes = true;
    addRequirements(shooter,spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooter.setVelocity(velocity);
    this.spinner.fuelRun();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooter.motionMagicSetVelocity();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooter.motionMagicSetVelocity();
    this.spinner.stopFuel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
