// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//SYSID IMPORT
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.Shooter.kTransferAngle;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Loader m_loader = new Loader();
  private final Elevator m_elevator = new Elevator();
  private final Turret m_turret = new Turret();
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(kDriverControllerPort);
      
      /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // LiveWindow.disableAllTelemetry();
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    autoChooser.addOption("Start_3_End_13_14_15", Autos.Start_3_End_13_14_15(m_drivetrain, m_intake, m_loader, m_turret, m_shooter));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
    
    m_drivetrain.setDefaultCommand(
      new DriveWithJoysticksFieldRelative(
        m_drivetrain, 
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightX(), 
        () -> m_driverController.getRightTriggerAxis()
        )
    );
    
    // m_shooter.setDefaultCommand(new RevShooter(m_drivetrain, m_shooter,  m_loader));

    // m_turret.setDefaultCommand(new SetTurretAngle(m_turret, kTransferAngle));

    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setIntake(0.), m_intake));

    m_loader.setDefaultCommand(new RunCommand(() -> m_loader.setLoaderMotor(0.), m_loader));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /** SYSID ROUTINE BUTTONS */
    // // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
    // // once.
    // m_driverController.a().whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.b().whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.x().whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.y().whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // m_driverController.start().onTrue(new RunCommand(() -> m_drivetrain.orchestra.play(), m_drivetrain)).onFalse(new RunCommand(() -> m_drivetrain.orchestra.stop(), m_drivetrain));

      // m_driverController.a().whileTrue(new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightTriggerAxis()));
    // m_driverController.a().whileTrue(new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> 0., () -> 0., () -> 0.));
    // m_driverController.a().whileTrue(new ShootNoteAtSpeedAndAngle(m_shooter, m_turret, m_loader, 4000., 35.));
    // m_driverController.y().whileTrue(new AlignWithAprilTag(m_drivetrain, 1.5));

      // m_driverController.leftTrigger().whileTrue(new AutoCollect(m_intake, m_turret, m_loader)).onFalse(new InstantCommand(() -> {
      //   m_intake.setIntake(0.);
      //   m_shooter.setShooterSpeed(0.);
      //   m_loader.setLoaderMotor(0.);
      // }));

      // m_driverController.back().whileTrue(m_drivetrain.getCommandToPathfindToPoint(Constants.PathfindingPoints.Red.CenterStage, 0.));
    // m_driverController.back().whileTrue(new DepositInAmp(m_drivetrain, m_intake, m_loader, m_turret, m_shooter));
    // m_driverController.back().whileTrue(new DriveForDistanceInDirection(m_drivetrain, 0., 0.5));

      // m_driverController.start().whileTrue(new ShootNoteAtSpeedAndAngle(m_shooter, m_turret, m_loader, 1000., 15.));
    // m_driverController.back().whileTrue(new ShootNoteAtSpeedAndAngle(m_shooter, m_turret, m_loader, 5676., 15.));
    // m_driverController.a()
    //   .onTrue(
    //   new InstantCommand(() -> {
    //     m_shooter.setRightShooterSpeed(5676.);
    //     m_shooter.setLeftShooterSpeed(5676.*.85);
    //   }, m_shooter).andThen(
    //     new WaitUntilCommand(() -> m_shooter.getRightShooterSpeed()>= 0.8 * 5676)
    //   ).andThen(new InstantCommand(() ->
    //     m_loader.setLoaderMotor(0.7), m_loader)
    //   )
    //   ).onFalse(
    //     new InstantCommand(() -> {
    //       m_shooter.setShooterSpeed(0.);
    //       m_loader.setLoaderMotor(0.);}, m_shooter, m_loader));
    
    // m_driverController.x().onTrue(new InstantCommand(() -> m_loader.setLoaderMotor(0.7), m_loader)).onFalse(new InstantCommand(() -> m_loader.setLoaderMotor(0.), m_loader));

      // m_driverController.b().onTrue(new InstantCommand(() -> {
      //   m_loader.setLoaderMotor(-0.5);
      //   m_intake.setIntake(-0.5);
      //   m_shooter.setShooterSpeed(-300.0);
      // }, m_shooter, m_intake, m_loader)).onFalse(new InstantCommand(() -> {
      //   m_loader.setLoaderMotor(0.0);
      //   m_intake.setIntake(0.0);
      //   m_shooter.setShooterSpeed(0.0);
      // }, m_shooter, m_intake, m_loader));

      // m_driverController.b().whileTrue(
      //   new DriveWithJoysticksRobotRelative(
      //     m_drivetrain, 
      //     () -> m_driverController.getLeftX(), 
      //     () -> m_driverController.getLeftY(), 
      //     () -> m_driverController.getRightX(), 
      //     () -> m_driverController.getRightTriggerAxis()
      //     )
      // );
    // m_driverController.x().whileTrue(new AutoTrackNote(m_drivetrain, m_intake)
    //   .andThen(new ParallelRaceGroup(
    //     new DriveWithJoysticks(
    //       m_drivetrain, 
    //       () -> m_driverController.getLeftX(), 
    //       () -> m_driverController.getLeftY(), 
    //       () -> m_driverController.getRightX(), 
    //       () -> m_driverController.getRightTriggerAxis(), 
    //       new Trigger(() -> false),
    //       new Trigger(() -> false),
    //       new Trigger(() -> false),
    //       new Trigger(() -> false)
          
    //       // m_driverController.povDown(),
    //       // m_driverController.b(),
    //       // m_driverController.leftBumper(),
    //       // m_driverController.rightBumper()
    //       ),
    //     new SequentialCommandGroup(
    //       new WaitCommand(0.2),
    //       new RunCommand(() -> m_intake.setIntake(0.3), m_intake).until(() -> m_intake.hasNoteInIntake()),
    //       new InstantCommand(() -> m_intake.setIntake(0.0), m_intake),
    //       new AutoTransfer(m_intake, m_shooter, m_turret, m_loader)
    //     )
    //   ))
      // m_driverController.x().whileTrue(new AutoTrackCollectNote(m_drivetrain, m_intake, m_loader, m_turret, m_shooter, m_driverController).until(() -> m_loader.hasNoteInShooter()))
      //   .onFalse(new InstantCommand(() -> {
      //   m_shooter.setShooterSpeed(0.);
      //   m_loader.setLoaderMotor(0.);
      //   m_intake.setIntake(0.);
      //   }, m_intake, m_shooter, m_turret, m_loader));
    
      // m_driverController.povUp().whileTrue(new SetTurretAngle(m_turret, 55.)).onFalse(new InstantCommand(() -> {}, m_shooter));
      // m_driverController.povDown().whileTrue(new SetTurretAngle(m_turret, 40.)).onFalse(new InstantCommand(() -> {}, m_shooter));
    
    m_driverController.povRight().onTrue(new InstantCommand(() -> m_loader.setLoaderMotor(0.4))).onFalse(new InstantCommand(() -> m_loader.setLoaderMotor(0.0)));
    m_driverController.povLeft().onTrue(new InstantCommand(() -> m_loader.setLoaderMotor(-0.4))).onFalse(new InstantCommand(() -> m_loader.setLoaderMotor(0.0)));

    m_driverController.a().onTrue(new InstantCommand(() -> m_intake.setIntake(0.6))).onFalse(new InstantCommand(() -> m_intake.stop()));
    m_driverController.povUp().onTrue(new InstantCommand(() -> m_intake.setAngleMotor(0.2))).onFalse(new InstantCommand(() -> m_intake.setAngleMotor(0.)));
    m_driverController.povDown().onTrue(new InstantCommand(() -> m_intake.setAngleMotor(-0.2))).onFalse(new InstantCommand(() -> m_intake.setAngleMotor(0.)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
