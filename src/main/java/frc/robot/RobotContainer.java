// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//SYSID IMPORT
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.PathfindingPoints.Blue;
import frc.robot.Constants.PathfindingPoints.Red;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.LEDs.kConfidentShotRange;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain();
  public final Shooter m_shooter = new Shooter();
  public final Intake m_intake = new Intake();
  public final IntakePivot m_intakePivot = new IntakePivot();
  public final Loader m_loader = new Loader();
  public final Elevator m_elevator = new Elevator();
  public final Turret m_turret = new Turret();
  public final Climber m_climber = new Climber();
  public final LEDs m_LEDs = new LEDs();
  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(kOperatorControllerPort);
      
      /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // LiveWindow.disableAllTelemetry();
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    autoChooser.addOption("Start_1_End_Upstage", Autos.Start_1_End_Upstage(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator));
    autoChooser.addOption("Start_3_End_2_1_12", Autos.Start_3_End_2_1_12(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator));
    autoChooser.addOption("Start_3_End_13_14_15", Autos.Start_3_End_13_14_15(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator));

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
    
    m_shooter.setDefaultCommand(new RevShooter(m_drivetrain, m_shooter,  m_loader));

    // m_turret.setDefaultCommand(new SetTurretAngle(m_turret, kTransferAngle));

    // m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));

    m_intake.setDefaultCommand(new IntakeDefault(m_intake, m_loader));

    m_intakePivot.setDefaultCommand(new IntakePivotDefault(m_intakePivot));

    // m_intakePivot.setDefaultCommand(new RunCommand(() -> m_intakePivot.setAngleMotor(0.008),  m_intakePivot));

    // m_intakePivot.setDefaultCommand(new IntakeToAngle(m_intakePivot, kMaxRotation));

    m_turret.setDefaultCommand(new SetTurretAngle(m_turret, 10.));

    m_loader.setDefaultCommand(new RunCommand(() -> m_loader.setLoaderMotor(0.), m_loader));

    m_elevator.setDefaultCommand(new RunCommand(() -> m_elevator.setElevator(kStallPower), m_elevator));

    m_LEDs.setDefaultCommand(new LEDDefault(m_LEDs, m_drivetrain, m_intake, m_loader));

    m_climber.setDefaultCommand(new RunCommand(() -> m_climber.runClimber(0.), m_climber));
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
    
    /********   DRIVER  CONTROLS   ********/
        
    m_driverController.rightBumper().whileTrue(
      new DriveWithJoysticksRobotRelative(
        m_drivetrain, 
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightX()
        )
    );
    
    m_driverController.leftBumper().whileTrue(new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader));

    // m_driverController.leftBumper().whileTrue(
    //   new ParallelCommandGroup(
    //     new TrackWhileMoving(m_drivetrain, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightTriggerAxis()),
    //     new RevShooter(m_drivetrain, m_shooter, m_loader),
    //     new AimAtSpeaker(m_turret, m_drivetrain)
    //     )
    // );
    // m_driverController.leftBumper().whileTrue(
    // new ConditionalCommand(
    //   new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader),  
    //   new AutoCollect(m_intakePivot, m_intake, m_turret, m_loader),
    //   () -> m_intake.hasNoteInIntake()
    //   ))
    // .onFalse(new InstantCommand(() -> {
    //   m_intake.setIntake(0.);
    //   m_shooter.setShooterSpeed(0.);
    //   m_loader.setLoaderMotor(0.);
    // }));
  
  
    m_driverController.a().whileTrue(new DepositInAmp(m_drivetrain, m_intake, m_loader, m_turret, m_shooter, m_elevator).unless(() -> !m_loader.hasNoteInShooter()))
    .onFalse(new ElevatorToHeight(m_elevator, kMinHeight));
  
    // m_driverController.b().whileTrue(new CollectFromSource(m_drivetrain, m_turret, m_shooter, m_loader, m_elevator).unless(() -> m_loader.hasNoteInShooter())) 
    // // TODO: TUNE THIS
    //   .onFalse(new InstantCommand(() -> {}, m_loader, m_shooter, m_turret));

    m_driverController.b().whileTrue(new RunCommand(() -> m_loader.setLoaderMotor(0.9), m_loader));

    m_driverController.x().whileTrue(new AutoTrackCollectNote(m_elevator, m_drivetrain, m_intake, m_intakePivot, m_loader, m_turret, m_shooter, m_driverController).until(() -> m_loader.hasNoteInShooter())
    .andThen(new ParallelCommandGroup(
      new TrackWhileMoving(m_drivetrain, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightTriggerAxis()),
      new RevShooter(m_drivetrain, m_shooter, m_loader),
      new AimAtSpeaker(m_turret, m_drivetrain)
      ).unless(() -> m_loader.hasNoteInShooter())))
      .onFalse(new InstantCommand(() -> {
      m_shooter.setShooterSpeed(0.);
      m_loader.setLoaderMotor(0.);
      m_intake.setIntake(0.);
      }, m_intake, m_shooter, m_turret, m_loader));
    
    m_driverController.y().whileTrue(new ParallelCommandGroup(new SequentialCommandGroup(
      new RunCommand(() -> {
        m_shooter.setLeftShooterSpeed(-3000.);
        m_shooter.setRightShooterSpeed(-2000.);
        m_loader.setLoaderMotor(-0.2);
      }, m_loader, m_shooter).until(() -> m_loader.hasNoteInShooter()), 
      new WaitUntilCommand(() -> !m_loader.hasNoteInShooter()), 
      new InstantCommand(() -> m_shooter.stopShooter()),
      new RunCommand(() -> m_loader.setLoaderMotor(0.1), m_loader).until(() -> m_loader.hasNoteInShooter()),
      new InstantCommand(() -> m_loader.stop())
    ),
    new SetTurretAngle(m_turret, kSourceAngle)));


    // m_driverController.povUp().whileTrue(
    //   m_drivetrain.getCommandToPathfindToPoint(DriverStation.getAlliance().get()==Alliance.Blue ? Blue.Source: Red.Source, 0.)
    //   .until(() -> m_drivetrain.getTV())
    // .andThen(new ConditionalCommand(
    //   new AutoTrackCollectNote(m_elevator, m_drivetrain, m_intake, m_intakePivot, m_loader, m_turret, m_shooter, m_driverController), 
    //   new CollectFromSource(m_drivetrain, m_turret, m_shooter, m_loader, m_elevator),
    //   () -> m_intake.getTV())) //TODO: TEST CONDITION FOR SOURCE PATHFINDING
    //   );
    
    // m_driverController.povDown().whileTrue(
    //   // TODO: TEST THIS
    //   m_drivetrain.getCommandToPathfindToPoint(DriverStation.getAlliance().get()==Alliance.Blue ? Blue.Speaker: Red.Speaker, 0.)
    //   .until(() -> m_drivetrain.getDistanceToSpeaker() <= kConfidentShotRange).unless(() -> !m_loader.hasNoteInShooter())
    //   .andThen(new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> 0., () -> 0., () -> 1.).unless(() -> !m_loader.hasNoteInShooter()))
    //   );
    // m_driverController.povUp().whileTrue(new RunCommand(() -> m_elevator.setElevator(0.2), m_elevator));

    // m_driverController.povDown().whileTrue(new RunCommand(() -> m_elevator.setElevator(-0.2), m_elevator));

    m_driverController.povUp().whileTrue(new AutoCollect(m_intakePivot, m_intake, m_turret, m_loader));
    m_driverController.povDown().whileTrue(new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader));

    m_driverController.povLeft().whileTrue(new RunCommand(() -> {
      m_intake.setIntake(-0.4);
      m_loader.setLoaderMotor(-0.8);
    }, m_intake, m_loader)).onFalse(new InstantCommand(() ->{
      m_intake.setIntake(0.);
      m_loader.stop();}, m_intake, m_loader));

    m_driverController.povRight().whileTrue(new RunCommand(() -> {
      m_intake.setIntake(0.4);
      m_loader.setLoaderMotor(0.8);
    }, m_intake, m_loader).until(() -> m_loader.hasNoteInShooter())).onFalse(new InstantCommand(() ->{
      m_intake.setIntake(0.);
      m_loader.stop();}, m_intake, m_loader));
      

    m_driverController.start().whileTrue(new ElevatorToHeight(m_elevator, kMaxHeight));

    m_driverController.back().whileTrue(new ElevatorToHeight(m_elevator, kMinHeight));
    
    /*********  END DRIVER CONTROLS  *********/
    /*-------------------------------------------------- */
    /*********   OPERATOR CONTROLS   *********/

    m_operatorController.leftTrigger().whileTrue(
        new ShootWhenClose(m_drivetrain, m_shooter, m_turret, m_loader, 
        //ZEROS ACT AS SHOOT WHILE STILL INSTEAD OF WHILE MOVING
        // () -> 0.,
        // () -> 0.,
        // () -> 0.
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightTriggerAxis()
        )
      );

    m_operatorController.leftBumper().whileTrue(new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> 0., () -> 0., () -> 0.));

    m_operatorController.rightBumper().whileTrue(AutoBuilder.followPath(PathPlannerPath.fromPathFile("PS3-IS3")));

    m_operatorController.axisGreaterThan(1, 0.4).whileTrue(new RunCommand(() -> m_elevator.setElevator(-0.3), m_elevator));
    m_operatorController.axisLessThan(1, -0.4).whileTrue(new RunCommand(() -> m_elevator.setElevator(0.3), m_elevator));

    m_operatorController.start().whileTrue(new RunCommand(() -> m_climber.runClimber(0.3), m_climber));

    m_operatorController.povLeft().whileTrue(new RunCommand(() -> m_turret.setAngleMotor(0.3), m_turret));
    m_operatorController.povRight().whileTrue(new RunCommand(() -> m_turret.setAngleMotor(-0.3), m_turret));

    m_operatorController.povUp().whileTrue(new RunCommand(() -> m_intakePivot.setAngleMotor(0.3), m_intakePivot));
    m_operatorController.povDown().whileTrue(new RunCommand(() -> m_intakePivot.setAngleMotor(-0.3), m_intakePivot));

    m_operatorController.b().whileTrue(new RunCommand(() -> m_loader.setLoaderMotor(0.8), m_loader));
    
    m_operatorController.a().whileTrue(new ShootNoteAtSpeedAndAngle(m_shooter, m_turret, m_loader, 5676 * 0.8, 52.));
    
    m_operatorController.x().whileTrue(new RunCommand(() -> m_loader.setLoaderMotor(-0.4), m_loader));

    m_operatorController.y().whileTrue(new SetTurretAngle(m_turret, 80.));

    m_operatorController.back().whileTrue(new ConditionalCommand(
        new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> 0., () -> 0., () -> 0.),
        new ParallelDeadlineGroup(
          new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader),
          new RevShooter(m_drivetrain, m_shooter, m_loader)).andThen(
            new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> 0., () -> 0., () -> 0.)).unless(() -> !m_intake.hasNoteInIntake()), 
        () -> m_loader.hasNoteInShooter()));
    
    /********* END OPERATOR CONTROLS *********/
    /*-------------------------------------------------- */
    /*********   TESTING  CONTROLS   *********/


    // m_driverController.y().whileTrue(new AutoTrackCollectNote(m_elevator, m_drivetrain, m_intake, m_intakePivot, m_loader, m_turret, m_shooter, m_driverController));

      // m_driverController.a().whileTrue(new CollectFromSource(m_drivetrain, m_turret, m_shooter, m_loader, m_elevator, m_driverController));

    // m_driverController.povLeft().whileTrue(new RunCommand(() -> {
    //   m_turret.setAngleMotor(0.3);
    // }, m_turret)).onFalse(new InstantCommand(() -> m_turret.stop()));
    
    // m_driverController.povRight().whileTrue(new RunCommand(() -> {
    //   m_turret.setAngleMotor(-0.3);
    // }, m_turret)).onFalse(new InstantCommand(() -> m_turret.stop()));
    

    
    // m_driverController.start().whileTrue(new ElevatorToHeight(m_elevator, kMaxHeight));

    // m_driverController.back().whileTrue(new ElevatorToHeight(m_elevator, kMinHeight));
    // m_driverController.povDown().whileTrue(new RunCommand(() -> m_intakePivot.setAngleMotor(-0.4), m_intakePivot));
    // m_driverController.povUp().whileTrue(new RunCommand(() -> m_intakePivot.setAngleMotor(0.4), m_intakePivot));

    // m_driverController.povDown().whileTrue(new IntakeToAngle(m_intakePivot, kMinRotation));

    // m_driverController.povUp().whileTrue(new IntakeToAngle(m_intakePivot, kMaxRotation));

    // m_driverController.x().whileTrue(new AutoCollect(m_intakePivot, m_intake, m_turret, m_loader)).onFalse(new InstantCommand(() -> {
    //   m_intake.stop();
    //   m_intakePivot.stop();
    //   m_loader.stop();
    //   m_turret.stop();
    // }, m_intake, m_intakePivot, m_turret, m_loader));
    
    
    // m_driverController.b().onTrue(new InstantCommand(() -> m_loader.setLoaderMotor(0.7), m_loader)).onFalse(new InstantCommand(() -> m_loader.stop(), m_loader));
    
    
    // m_driverController.a().whileTrue(new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader)).onFalse(new InstantCommand(() -> {
      //   m_intake.stop();
      //   m_intakePivot.stop();
      //   m_loader.stop();
      //   m_turret.stop();
      // }, m_intake, m_intakePivot, m_turret, m_loader));


    // m_driverController.leftTrigger().whileTrue(new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> 0., () -> 0., () -> 0.));

    // m_driverController.x().whileTrue(new AutoCollect(m_intakePivot, m_intake, m_turret, m_loader));
    // m_driverController.y().whileTrue(new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader));


    // m_driverController.leftBumper().whileTrue(new AutoTrackCollectNote(m_elevator, m_drivetrain, m_intake, m_intakePivot, m_loader, m_turret, m_shooter, m_driverController));
    // m_driverController.leftBumper().whileTrue(new ShootNoteAtSpeed(m_shooter, m_loader, 2000.0, m_driverController.b(), new Trigger(() -> false), new Trigger(() -> false)));
    
    
    // m_driverController.povLeft().whileTrue(new RunCommand(() -> m_intake.setIntake(-0.5), m_intake));

    // m_driverController.y().whileTrue(new ShootNoteAtSpeed(m_shooter, m_loader, 3000., null, null, null))

    // m_driverController.x().whileTrue(new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader));
    // m_driverController.x().whileTrue(new RunCommand(() -> m_intakePivot.setAngleMotor(-0.3), m_intakePivot));
    // m_driverController.y().whileTrue(new RunCommand(() -> m_intakePivot.setAngleMotor(.3), m_intakePivot));


    // m_driverController.a().whileTrue(new AutoCollect(m_intakePivot, m_intake, m_turret, m_loader));


    // m_driverController.y()
    // m_driverController.a().whileTrue(new RunCommand(() -> m_intake.setIntake(1.), m_intake));
    // m_driverController.b().whileTrue(new RunCommand(() -> m_intake.setIntake(-1.), m_intake));



    // m_driverController.leftTrigger().whileTrue(new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightTriggerAxis()));
    // // m_driverController.rightBumper().whileTrue(new ShootNoteAtSpeedAndAngle(m_shooter, m_turret, m_loader, 5676., 25.));
    // // m_driverController.y().onTrue(new InstantCommand(() -> m_shooter.setShooterSpeed(3000.), m_shooter)).onFalse(new InstantCommand(() -> m_shooter.setShooterSpeed(0.), m_shooter));
    // m_driverController.y().whileTrue(new SetTurretAngle(m_turret, 80.));
    // m_driverController.a().whileTrue(new InstantCommand(() -> m_climber.runClimber(0.3))).onFalse(new InstantCommand(() -> m_climber.runClimber(0.)));
    


    // m_driverController.povRight().whileTrue(new RunCommand(() -> m_intake.setIntake(0.3), m_intake));
    // m_driverController.povLeft().whileTrue(new RunCommand(() -> m_intake.setIntake(-0.3), m_intake));

    /********* END TESTING CONTROLS *********/
    /*-------------------------------------------------- */
    /******** SYSID ROUTINE BUTTONS *********/
          // // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
          // // once.
          // m_driverController.a().whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
          // m_driverController.b().whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
          // m_driverController.x().whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
          // m_driverController.y().whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    
          // m_driverController.a().whileTrue(new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), () -> m_driverController.getRightTriggerAxis()));
          // m_driverController.x().whileTrue(m_drivetrain.getCommandToPathfindToPoint(Red.Amp, 0.).andThen(new AlignWithAprilTag(m_drivetrain, .4)));
          // m_driverController.x().whileTrue(new DriveToPoint(m_drivetrain, Red.Amp));
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
