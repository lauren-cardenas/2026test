// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.ShootOnlyCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SpindexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class RobotContainer {

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final SpindexerSubsystem m_spindexerSubsystem = new SpindexerSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  //private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

  private final SendableChooser<Command> autoChooser;

 @SuppressWarnings("unused")
private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
 private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
      

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {

      NamedCommands.registerCommand("Shoot", new ShooterCommand(SpeedConstants.kAutoVelocity,m_shooterSubsystem,m_spindexerSubsystem));
      NamedCommands.registerCommand("IntakeDown", new RunCommand(() -> m_pivotSubsystem.manualPivot(SpeedConstants.kPivotSpeed), m_pivotSubsystem));
      NamedCommands.registerCommand("Rollers", new RunCommand(() -> m_intakeSubsystem.runRollers(SpeedConstants.kRollerSpeed), m_intakeSubsystem));
      NamedCommands.registerCommand("StopShoot", new ShooterCommand(0.1, m_shooterSubsystem, m_spindexerSubsystem));
      NamedCommands.registerCommand("StopIntake", new RunCommand(() -> m_pivotSubsystem.manualPivot(0), m_pivotSubsystem));
      NamedCommands.registerCommand("StopRollers", new RunCommand(() -> m_intakeSubsystem.runRollers(0), m_intakeSubsystem));
      
        configureBindings();

      autoChooser = AutoBuilder.buildAutoChooser("Outpost");
      SmartDashboard.putData("Auto Mode", autoChooser);
      CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

      
      
      
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

         /******************Driver Controller*************************/

    //Intake Rollers In
    m_operatorController.x().whileTrue(new RunCommand(() -> m_intakeSubsystem.runRollers(SpeedConstants.kRollerSpeed), m_intakeSubsystem))
      .onFalse(new InstantCommand(() -> m_intakeSubsystem.runRollers(0)));
    //Intake Rollers Out
    m_operatorController.b().whileTrue(new RunCommand(() -> m_intakeSubsystem.runRollers(-SpeedConstants.kRollerSpeed), m_intakeSubsystem))
      .onFalse(new InstantCommand(() -> m_intakeSubsystem.runRollers(0)));
    //Intake Pivot Down
    //m_driverController.a().whileTrue(new PivotCommand(PositionConstants.kIntakeDown, m_pivotSubsystem))
    //  .onFalse(new InstantCommand(() -> m_pivotSubsystem.manualPivot(0)));
    //Intake Pivot Up
    //m_driverController.y().whileTrue(new PivotCommand(PositionConstants.kIntakeUp, m_pivotSubsystem))
    //  .onFalse(new InstantCommand(() -> m_pivotSubsystem.manualPivot(0)));
    //m_driverController.a().whileTrue(new RunCommand(() -> m_pivotSubsystem.manualPivot(SpeedConstants.kPivotSpeed), m_intakeSubsystem))
      //.onFalse(new InstantCommand(() -> m_pivotSubsystem.manualPivot(0.0)));
    //m_driverController.y().whileTrue(new RunCommand(() -> m_pivotSubsystem.manualPivot(-SpeedConstants.kPivotSpeed), m_intakeSubsystem))
      //.onFalse(new InstantCommand(() -> m_pivotSubsystem.manualPivot(0.0)));
     
      
      /*****************Operator Controller***********************/

    //Motion Magic Shooter
    
    m_operatorController.y().whileTrue(new RunCommand(() -> m_pivotSubsystem.manualPivot(-SpeedConstants.kPivotSpeed), m_intakeSubsystem))
      .onFalse(new InstantCommand(() -> m_pivotSubsystem.manualPivot(0.0)));

    m_operatorController.a().whileTrue(new RunCommand(() -> m_pivotSubsystem.manualPivot(SpeedConstants.kPivotSpeed), m_intakeSubsystem))
      .onFalse(new InstantCommand(() -> m_pivotSubsystem.manualPivot(0.0)));

    m_operatorController.rightTrigger().whileTrue(new ShootOnlyCommand(SpeedConstants.kTrenchVelocity, m_shooterSubsystem))
      .onFalse(new InstantCommand(() -> m_shooterSubsystem.manualSpeed(0.1)));
      //.onFalse(new InstantCommand(() -> m_spindexerSubsystem.stopFuel()));

    m_operatorController.leftTrigger().whileTrue(new ShootOnlyCommand(SpeedConstants.kMaxSpeed, m_shooterSubsystem))
      .onFalse(new InstantCommand(() -> m_shooterSubsystem.manualSpeed(0.1)));

    
    /*m_operatorController.rightTrigger().whileTrue(new ShooterCommand(SpeedConstants.kTrenchVelocity, m_shooterSubsystem, m_spindexerSubsystem)) //FIXME Need limelight speed calculation  
      .onFalse(new InstantCommand(() -> m_shooterSubsystem.manualSpeed(0.1)))
      .onFalse(new InstantCommand(() -> m_spindexerSubsystem.stopFuel())); */

    /*
    final double speed = m_limelightSubsystem.getSpeed();
    m_operatorController.rightBumper().whileTrue(new ShooterCommand(speed, m_shooterSubsystem, m_spindexerSubsystem)) //FIXME Need limelight speed calculation  
      .onFalse(new InstantCommand(() -> m_shooterSubsystem.manualSpeed(0.1)))
      .onFalse(new InstantCommand(() -> m_spindexerSubsystem.stopFuel()));*/

    m_operatorController.rightBumper().whileTrue(new ShootOnlyCommand(SpeedConstants.kAutoVelocity, m_shooterSubsystem))
      .onFalse(new InstantCommand(() -> m_shooterSubsystem.manualSpeed(0)));
      //.onFalse(new InstantCommand(() -> m_spindexerSubsystem.stopFuel()));

    m_operatorController.povLeft().whileTrue(new RunCommand(() -> m_turretSubsystem.turnLeft(Constants.TurretConstants.tempSpeed)))
      .onFalse(new InstantCommand(() -> m_turretSubsystem.stop()));

    m_operatorController.povRight().whileTrue(new RunCommand(() -> m_turretSubsystem.turnRight(-Constants.TurretConstants.tempSpeed)))
          .onFalse(new InstantCommand(() -> m_turretSubsystem.stop()));
    
    m_operatorController.leftBumper().whileTrue(new RunCommand(() -> m_spindexerSubsystem.fuelRun()))
      .onFalse(new InstantCommand(() -> m_spindexerSubsystem.stopFuel()));

        }
        
        @SuppressWarnings("unused")
        private Object m_turretSubsystem(double d) {
          // TODO Auto-generated method stub
          throw new UnsupportedOperationException("Unimplemented method 'm_turretSubsystem'");
        }
    /* 
        public Command getDriveAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }*/
    public Command getAutonomousCommand(){
      return autoChooser.getSelected();
    }
}
