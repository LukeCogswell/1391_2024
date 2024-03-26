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
import static frc.robot.Constants.MeasurementConstants.kFieldX;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  // public final Climber m_climber = new Climber();
  public final LEDs m_LEDs = new LEDs();
  private final SendableChooser<Command> autoChooser;

  private Trigger transferTrigger = new Trigger(() -> m_intake.hasNoteInIntake() && !m_loader.hasNoteInShooter()); 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
    new CommandXboxController(kDriverControllerPort);

  private final CommandXboxController m_operatorController =
    new CommandXboxController(1);
      
      /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // LiveWindow.disableAllTelemetry();
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.addOption("ChoreoStart_DownSpeakerSWM_End_15_14", Autos.Choreo_Start_DownSpeakerSWM_End_15_14(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator, m_LEDs));
    autoChooser.addOption("Start_DownSpeakerSWM_End_15_14", Autos.Start_DownSpeakerSWM_End_15_14(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator, m_LEDs));
    autoChooser.addOption("Start_DownSpeakerSWM_End_15_13", Autos.Start_DownSpeakerSWM_End_15_13(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator, m_LEDs));
    autoChooser.addOption("Start_Source_End_15_14", Autos.Start_Source_End_15_14(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator, m_LEDs));
    autoChooser.addOption("Start_Source_End_14_13", Autos.Start_Source_End_14_13(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator, m_LEDs));
    autoChooser.addOption("Start_1_End_Upstage", Autos.Start_1_End_Upstage(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator, m_LEDs));
    autoChooser.addOption("Start_3_End_2_1_12", Autos.Start_3_End_2_1_12(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator, m_LEDs));
    autoChooser.addOption("Start_3_End_13_14_15", Autos.Start_3_End_13_14_15(m_drivetrain, m_intakePivot, m_intake, m_loader, m_turret, m_shooter, m_elevator, m_LEDs));
    autoChooser.addOption("Drive Straight", Autos.DriveStraight(m_drivetrain, m_shooter, m_turret, m_loader, m_LEDs));

    Shuffleboard.getTab("Matches").add("Auto Chooser", autoChooser).withPosition(4, 0).withSize(4, 1);
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

    // m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));

    m_shooter.setDefaultCommand(new RunCommand(
      () -> {
        m_shooter.setShooterSpeed(m_loader.hasNoteInShooter() ? 5676. : 0.);
      }, m_shooter));

    // m_turret.setDefaultCommand(new SetTurretAngle(m_turret, kTransferAngle));

    // m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));

    m_intake.setDefaultCommand(new IntakeDefault(m_intake, m_loader));

    m_intakePivot.setDefaultCommand(new IntakePivotDefault(m_intakePivot));

    // m_intakePivot.setDefaultCommand(new RunCommand(() -> m_intakePivot.setAngleMotor(0.00),  m_intakePivot));

    // m_intakePivot.setDefaultCommand(new IntakeToAngle(m_intakePivot, kMaxRotation));

    m_turret.setDefaultCommand(new SetTurretAngle(m_turret, 13.));
    //m_turret.setDefaultCommand(new RunCommand(() -> m_turret.stop(), m_turret));

    m_loader.setDefaultCommand(new RunCommand(() -> m_loader.setLoaderMotor(0.), m_loader));

    m_elevator.setDefaultCommand(new RunCommand(() -> m_elevator.setElevator(kStallPower), m_elevator));

    m_LEDs.setDefaultCommand(new LEDDefault(m_LEDs, m_drivetrain, m_intake, m_loader, m_turret));

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
    
    // m_driverController.a().whileTrue(new RunCommand(() -> {
    //   m_turret.setMotor(1.);
    // }, m_turret));

    /********   DRIVER  CONTROLS   ********/
    transferTrigger.whileTrue(new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader).unless(() -> m_loader.hasNoteInShooter()));
        
    m_driverController.start().whileTrue(new RunCommand(() -> {
      if (m_drivetrain.getTV()){
        m_drivetrain.driveAroundPoint(0., 0., 0.2, true, new Translation2d(m_drivetrain.getDistanceToSpeakerAprilTag(), m_drivetrain.getOdometryYaw()));
      }
    }, m_drivetrain));
    // m_driverController.start().whileTrue(new RunCommand(() -> m_loader.setLoaderMotor(1.)));
    // m_driverController.start().whileTrue(
    //   new ParallelCommandGroup(
    //     new IntakeToAngle(m_intakePivot, kMinRotation),
    //     new RunCommand(() -> m_intake.setIntake(1.5), m_intake),
    //     new RunCommand(() -> m_shooter.setShooterSpeed(-1000.), m_shooter)
    //   )
    // );

    m_driverController.leftBumper().whileTrue(new ShootWhileMoving(m_drivetrain, m_shooter, m_turret, m_loader, () -> 0., () -> 0., () -> 0., m_LEDs));

    m_driverController.rightBumper().whileTrue(
      new DriveWithJoysticksRobotRelative(
        m_drivetrain, 
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightX()
        )
    );

    m_driverController.x().whileTrue(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new AutoTrackNote(m_drivetrain, m_intake, m_intakePivot),
        new IntakeToAngle(m_intakePivot, kMinRotation)),
      new IntakeToAngle(m_intakePivot, kMaxRotation).until(() -> m_intakePivot.getIntakeAngle() <= 250.),
      new ParallelDeadlineGroup(
        new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader),
        new DriveWithJoysticksFieldRelative(
        m_drivetrain, 
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightX(), 
        () -> m_driverController.getRightTriggerAxis()
        ))));
    
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

    // m_driverController.povDown().onTrue(new InstantCommand(() -> m_drivetrain.setLimelightTargetID(7.)));

    m_driverController.povLeft().whileTrue(new RunCommand(() -> {
      m_intake.setIntake(-0.4);
      m_loader.setLoaderMotor(-1.);
    }, m_intake, m_loader)).onFalse(new InstantCommand(() ->{
      m_intake.setIntake(0.);
      m_loader.stop();}, m_intake, m_loader));

    m_driverController.povRight().whileTrue(new RunCommand(() -> {
      m_intake.setIntake(0.4);
      m_loader.setLoaderMotor(0.8);
    }, m_intake, m_loader).until(() -> m_loader.hasNoteInShooter())).onFalse(new InstantCommand(() ->{
      m_intake.setIntake(0.);
      m_loader.stop();}, m_intake, m_loader));
      

    m_driverController.povUp().whileTrue(new SetTurretAngle(m_turret, 50.));
    m_driverController.povDown().whileTrue(new ElevatorToHeight(m_elevator, 6.));
    // m_driverController.leftTrigger().whileTrue(new SetTurretAngle(m_turret, 20.));
    m_driverController.back().onTrue(new InstantCommand(() -> m_drivetrain.setFieldPosition(new Pose2d(m_drivetrain.getFieldPosition().getX(), m_drivetrain.getFieldPosition().getY(), new Rotation2d(DriverStation.getAlliance().get()==Alliance.Blue ? 0. : Math.PI)))));
    
    // m_driverController.start().whileTrue(new ElevatorToHeight(m_elevator, kMaxHeight));
    // m_driverController.back().whileTrue(new ElevatorToHeight(m_elevator, kMinHeight));
    
    /*********  END DRIVER CONTROLS  *********/
    /*-------------------------------------------------- */
    /*********   OPERATOR CONTROLS   *********/

    m_operatorController.rightTrigger().whileTrue(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader).until(() -> m_loader.hasNoteInShooter()),
        new RunCommand(() -> m_shooter.setShooterSpeed(4000.), m_shooter)),
      new PrepToShootFromSetpoint(4000., 50., DriverStation.getAlliance().get() == Alliance.Blue ? -30. : -180 + 30, m_drivetrain, m_shooter, m_turret, m_loader, m_LEDs, 
        m_driverController.leftTrigger(), () -> m_driverController.getLeftX(),() -> m_driverController.getLeftY(), () -> m_driverController.getRightTriggerAxis())));

    m_operatorController.rightBumper().whileTrue(new ShootNoteAtSpeed(m_shooter, m_loader, 950., m_driverController.leftTrigger(), new Trigger(() -> false), new Trigger(() -> false), true));

    m_operatorController.leftBumper().whileTrue(new ParallelCommandGroup(
      new ElevatorToHeight(m_elevator, 4.),
      new SetTurretAngle(m_turret, 175.)
    ).until(() -> m_turret.getShooterAngle() >= 150.).andThen(new ParallelCommandGroup(
      new ElevatorToHeight(m_elevator, 6.5),
      new SetTurretAngle(m_turret, 175.)
    ))).onFalse(new RunCommand(() -> m_turret.stop(), m_turret));

    // m_operatorController.back().whileTrue(new InstantCommand(() -> m_drivetrain.setFieldPosition(new Pose2d(kFieldX - 2, 7, new Rotation2d(Math.PI)))).andThen(AutoBuilder.followPath(PathPlannerPath.fromPathFile("STRAIGHT"))));
    // m_operatorController.back().whileTrue(new ShootNoteAtSpeedAndAngle(m_shooter, m_turret, m_loader, 2500., -27.));
    m_operatorController.back().whileTrue(new RunCommand(() -> m_elevator.setElevator(-.5), m_elevator));

    m_operatorController.start().whileTrue(new DepositInAmp(m_drivetrain, m_intake, m_loader, m_turret, m_shooter, m_elevator, m_driverController.leftTrigger()));
    
    m_operatorController.axisGreaterThan(1, 0.4).whileTrue(new RunCommand(() -> m_elevator.setElevator(-0.3), m_elevator));
    // m_operatorController.axisLessThan(1, -0.4).whileTrue(new RunCommand(() -> m_elevator.setElevator(0.3), m_elevator));

    m_operatorController.povLeft().whileTrue(new RunCommand(() -> m_turret.setAngleMotor(0.3), m_turret)).onFalse(new RunCommand(() -> m_turret.stop(), m_turret));
    m_operatorController.povRight().whileTrue(new RunCommand(() -> m_turret.setAngleMotor(-0.3), m_turret)).onFalse(new RunCommand(() -> m_turret.stop(), m_turret));

    m_operatorController.povUp().whileTrue(new RunCommand(() -> m_intakePivot.setAngleMotor(0.3), m_intakePivot));
    m_operatorController.povDown().whileTrue(new RunCommand(() -> m_intakePivot.setAngleMotor(-0.3), m_intakePivot));

    m_operatorController.x().whileTrue(new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader));
    
    m_operatorController.a().whileTrue(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader).until(() -> m_loader.hasNoteInShooter()),
        new RunCommand(() -> m_shooter.setShooterSpeed(5676 * 0.6), m_shooter)),
      new ParallelCommandGroup(
      new ShootSpeedAngleWithControl(m_shooter, m_turret, m_loader, 5676 * 0.6, 53.5, m_driverController.leftTrigger(), false),
      new IntakeToAngle(m_intakePivot, 210.))));
    m_operatorController.b().whileTrue(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader).until(() -> m_loader.hasNoteInShooter()),
        new RunCommand(() -> m_shooter.setShooterSpeed(5676 * 0.75), m_shooter)),
      new PrepToShootFromSetpoint(
        5676 * 0.75,
        31.,
        DriverStation.getAlliance().get() == Alliance.Blue ? -31 : (-180 + 31.),
        m_drivetrain, m_shooter, m_turret, m_loader, m_LEDs, 
        m_driverController.leftTrigger(), 
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightTriggerAxis())));
    m_operatorController.y().whileTrue(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new AutoTransfer(m_intakePivot, m_intake, m_elevator, m_turret, m_loader).until(() -> m_loader.hasNoteInShooter()),
        new RunCommand(() -> m_shooter.setShooterSpeed(5676 * 1.), m_shooter)),
      new PrepToShootFromSetpoint(
        5676 * 1.,
        17.5,
        DriverStation.getAlliance().get() == Alliance.Blue ? 0. : 180.,
        m_drivetrain, m_shooter, m_turret, m_loader, m_LEDs,
        m_driverController.leftTrigger(), 
        () -> m_driverController.getLeftX(), 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightTriggerAxis())));
    
    /********* END OPERATOR CONTROLS *********/
    /*-------------------------------------------------- */
    /*********   TESTING  CONTROLS   *********/
    
    
    // m_operatorController.y().whileTrue(new SetTurretAngle(m_turret, 80.));
    


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
