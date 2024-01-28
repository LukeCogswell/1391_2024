// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignWithAprilTag;
import frc.robot.commands.AutoCollect;
import frc.robot.commands.AutoTrackNote;
import frc.robot.commands.AutoTransfer;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.EjectNotes;
import frc.robot.commands.SetTurretAngle;
import frc.robot.commands.ShootNoteAtSpeedAndAngle;
import frc.robot.commands.ShootWhileMoving;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  private final Turret m_turret = new Turret();
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
      
      /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // LiveWindow.disableAllTelemetry();
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
  
    NamedCommands.registerCommand("AutoCollect", new AutoCollect(m_intake, m_turret, m_loader));

    NamedCommands.registerCommand("Intake Down", new InstantCommand(() -> {
      m_intake.setIntake(0.7);
      m_intake.setIntake(0.5);
      m_loader.setLoaderMotor(0.7);
      m_shooter.setShooterSpeed(-400.0); 
      }, m_shooter, m_intake, m_loader));

    NamedCommands.registerCommand("Intake Up", new InstantCommand(() -> {
      m_intake.setIntake(0.);
      m_intake.setIntake(0.);
      m_loader.setLoaderMotor(0.);
      m_shooter.setShooterSpeed(0.);
    }, m_shooter, m_intake, m_loader));

    NamedCommands.registerCommand("Spin Up Shooter", new InstantCommand(() -> m_shooter.setShooterSpeed(3000.)));

    NamedCommands.registerCommand("Transfer", new SetTurretAngle(m_turret, 15.).withTimeout(1));

    NamedCommands.registerCommand("Shoot 45", new ShootNoteAtSpeedAndAngle(m_shooter, m_turret, m_loader, 3000., 40.).withTimeout(2));
    NamedCommands.registerCommand("Shoot 50", new ShootNoteAtSpeedAndAngle(m_shooter, m_turret, m_loader, 3000., 55.).withTimeout(2));


    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    autoChooser.addOption("4 Piece", AutoBuilder.buildAuto("4 Piece"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
    
    m_drivetrain.setDefaultCommand(
      new DriveWithJoysticks(
        m_drivetrain, 
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightX(), 
        () -> m_driverController.getRightTriggerAxis(), 
        new Trigger(() -> false),
        new Trigger(() -> false),
        new Trigger(() -> false),
        new Trigger(() -> false)
        
        // m_driverController.povDown(),
        // m_driverController.b(),
        // m_driverController.leftBumper(),
        // m_driverController.rightBumper()
        )
    );
    
    m_turret.setDefaultCommand(new SetTurretAngle(m_turret, 15.));

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

    m_driverController.a().whileTrue(new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightTriggerAxis()));
    
    // m_driverController.y().whileTrue(new AlignWithAprilTag(m_drivetrain, 1.5));

    m_driverController.leftTrigger().whileTrue(new AutoCollect(m_intake, m_turret, m_loader)).onFalse(new InstantCommand(() -> {
      m_intake.setIntake(0.);
      m_shooter.setShooterSpeed(0.);
      m_loader.setLoaderMotor(0.);
    }));

    m_driverController.start().whileTrue(new ShootNoteAtSpeedAndAngle(m_shooter, m_turret, m_loader, 3000., 40.).withTimeout(2));

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

    m_driverController.b().onTrue(new InstantCommand(() -> {
      m_loader.setLoaderMotor(-0.5);
      m_intake.setIntake(-0.5);
      m_shooter.setShooterSpeed(-300.0);
    }, m_shooter, m_intake, m_loader)).onFalse(new InstantCommand(() -> {
      m_loader.setLoaderMotor(0.0);
      m_intake.setIntake(0.0);
      m_shooter.setShooterSpeed(0.0);
    }, m_shooter, m_intake, m_loader));

    m_driverController.x().whileTrue(new AutoTrackNote(m_drivetrain, m_intake)
      .andThen(new ParallelRaceGroup(
        new DriveWithJoysticks(
          m_drivetrain, 
          () -> m_driverController.getLeftX(), 
          () -> m_driverController.getLeftY(), 
          () -> m_driverController.getRightX(), 
          () -> m_driverController.getRightTriggerAxis(), 
          new Trigger(() -> false),
          new Trigger(() -> false),
          new Trigger(() -> false),
          new Trigger(() -> false)
          
          // m_driverController.povDown(),
          // m_driverController.b(),
          // m_driverController.leftBumper(),
          // m_driverController.rightBumper()
          ),
        new SequentialCommandGroup(
          new WaitCommand(0.2),
          new RunCommand(() -> m_intake.setIntake(0.3), m_intake).until(() -> m_intake.hasNoteInIntake()),
          new InstantCommand(() -> m_intake.setIntake(0.0), m_intake),
          new AutoTransfer(m_intake, m_shooter, m_turret, m_loader)
        )
      ))
      ).onFalse(new InstantCommand(() -> {
       m_shooter.setShooterSpeed(0.);
       m_loader.setLoaderMotor(0.);
       m_intake.setIntake(0.);
      }, m_intake, m_shooter, m_turret, m_loader));
      
    m_driverController.povUp().whileTrue(new SetTurretAngle(m_turret, 60.)).onFalse(new InstantCommand(() -> {}, m_shooter));
    m_driverController.povDown().whileTrue(new SetTurretAngle(m_turret, 0.)).onFalse(new InstantCommand(() -> {}, m_shooter));
    
    m_driverController.povRight().onTrue(new InstantCommand(() -> m_loader.setLoaderMotor(0.4))).onFalse(new InstantCommand(() -> m_loader.setLoaderMotor(0.0)));
    m_driverController.povLeft().onTrue(new InstantCommand(() -> m_loader.setLoaderMotor(-0.4))).onFalse(new InstantCommand(() -> m_loader.setLoaderMotor(0.0)));


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
