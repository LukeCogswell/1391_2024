// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.SetShooterAngle;
import frc.robot.commands.ShootNoteAtSpeed;
import frc.robot.commands.TrackWhileMoving;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.Constants.MeasurementConstants.*;

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
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
      
      /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
  
    NamedCommands.registerCommand("Intake Down", new InstantCommand(() -> {
      m_intake.setIntake(0.7);

      }));
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
  
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    // m_driverController.x().onTrue(new InstantCommand(() -> m_drivetrain.setFieldPosition(new Pose2d(56.5 / kInchesToMeters, 171 / kInchesToMeters, new Rotation2d(0.0)))));
    // m_driverController.b().onTrue(new InstantCommand(() -> m_drivetrain.setFieldPosition(new Pose2d(kFieldX - 56.5 / kInchesToMeters, kFieldY - 171 / kInchesToMeters, new Rotation2d(Math.PI)))));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.a().whileTrue(new TrackWhileMoving(m_drivetrain, m_shooter, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightTriggerAxis()));

    m_driverController.y().whileTrue(new ShootNoteAtSpeed(m_shooter, 5676.0));
    
    // m_driverController.y().whileTrue(m_drivetrain.getCommandToPathfindToPoint(Constants.PathfindingPoints.Red.CenterStage, 0.0));
    // m_driverController.povUp().onTrue(new InstantCommand(() -> m_intake.setIntake(0.4))).onFalse(new InstantCommand(() -> m_intake.setIntake(0.0)));
    // m_driverController.povDown().onTrue(new InstantCommand(() -> m_intake.setIntake(-0.4))).onFalse(new InstantCommand(() -> m_intake.setIntake(0.0)));


    m_driverController.leftTrigger().onTrue(new InstantCommand(() -> {
      m_shooter.setLoaderMotor(0.5);
      m_intake.setIntake(0.5);
      m_shooter.setShooterSpeed(-400.0);
    }, m_shooter, m_intake)).onFalse(new InstantCommand(() -> {
      m_shooter.setLoaderMotor(0.0);
      m_intake.setIntake(0.0);
      m_shooter.setShooterSpeed(0.0);
    }, m_shooter, m_intake));

    m_driverController.b().onTrue(new InstantCommand(() -> {
      m_shooter.setLoaderMotor(-0.5);
      m_intake.setIntake(-0.5);
      m_shooter.setShooterSpeed(-300.0);
    }, m_shooter, m_intake)).onFalse(new InstantCommand(() -> {
      m_shooter.setLoaderMotor(0.0);
      m_intake.setIntake(0.0);
      m_shooter.setShooterSpeed(0.0);
    }, m_shooter, m_intake));

    m_driverController.povUp().onTrue(new SetShooterAngle(m_shooter, 45.0)).onFalse(new InstantCommand(() -> {}, m_shooter));
    m_driverController.povDown().onTrue(new SetShooterAngle(m_shooter, 9.5)).onFalse(new InstantCommand(() -> {}, m_shooter));

    // m_driverController.povUp().onTrue(new InstantCommand(() -> m_shooter.setTopShooterSpeed(5676.0))).onFalse(new InstantCommand(() -> m_shooter.setTopShooterSpeed(0.0)));
    // m_driverController.povUp().onTrue(new InstantCommand(() -> m_shooter.setBottomShooterSpeed(5676.0))).onFalse(new InstantCommand(() -> m_shooter.setBottomShooterSpeed(0.0)));
    // m_driverController.povUp().onTrue(new InstantCommand(() -> m_shooter.setTopShooterSpeed(300.0))).onFalse(new InstantCommand(() -> m_shooter.setTopShooterSpeed(0.0)));
    // m_driverController.povDown().onTrue(new InstantCommand(() -> m_shooter.setBottomShooterSpeed(300.0))).onFalse(new InstantCommand(() -> m_shooter.setBottomShooterSpeed(0.0)));
    // m_driverController.start().onTrue(new InstantCommand(() -> m_shooter.setLoaderMotor(0.7))).onFalse(new InstantCommand(() -> m_shooter.setLoaderMotor(0.0)));

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
