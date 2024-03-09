// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.MeasurementConstants.kFieldX;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  // public static Command exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command Start_DownSpeaker_End_15_14(Drivetrain drivetrain, IntakePivot intakePivot, Intake intake, Loader loader, Turret turret, Shooter shooter, Elevator elevator, LEDs leds) {
    PathPlannerPath path0 = PathPlannerPath.fromPathFile("SDownSpeaker-EShoot");
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("SShoot-E15");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("S15-EDownstage");
    PathPlannerPath path3 = PathPlannerPath.fromPathFile("SDownstage-E14");
    PathPlannerPath path4 = PathPlannerPath.fromPathFile("S14-EDownstage");

    return Commands.sequence(
      new InstantCommand(
        () -> {
          if (DriverStation.getAlliance().get() == Alliance.Blue){
            drivetrain.setFieldPosition(new Pose2d(new Translation2d(0.66, 4.39), new Rotation2d(-60 * Math.PI / 180)));
          } else {
            drivetrain.setFieldPosition(new Pose2d(new Translation2d(kFieldX - 0.66, 4.39), new Rotation2d(-120 * Math.PI / 180)));
          }
        }
        ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path0),
        new SetTurretAngle(turret, 34.9),
        new RunCommand(() -> shooter.setShooterSpeed(5676.), shooter)),
      new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path1).until(() -> loader.hasNoteInShooter()),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      ),
      // new WaitCommand(0.5),
      // new AutoTransfer(intakePivot, intake, elevator, turret, loader).withTimeout(1.5),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path2),
        new RunCommand(() -> shooter.setShooterSpeed(5676.), shooter),
        // new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        new AutoTransfer(intakePivot, intake, elevator, turret, loader).until(() -> !intake.hasNoteInIntake() && loader.hasNoteInShooter()).andThen(
          new ParallelCommandGroup(
            new SetTurretAngle(turret, 23.),
            // new AimAtSpeaker(turret, drivetrain),
            new InstantCommand(() -> {
              loader.stop();
              intake.stop();
            }, loader, intake)))
      ),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter()),
      //END SHOOT OR TRANSFER THEN SHOOT ------
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path3).until(() -> loader.hasNoteInShooter()),
        new RunCommand(() -> shooter.setShooterSpeed(5676.)),
        // new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
        ),
        // new AutoTransfer(intakePivot, intake, elevator, turret, loader).withTimeout(1.5),
        new ParallelDeadlineGroup(
          AutoBuilder.followPath(path4),
          new RunCommand(() -> shooter.setShooterSpeed(5676.)),
        // new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        new AutoTransfer(intakePivot, intake, elevator, turret, loader).until(() -> !intake.hasNoteInIntake() && loader.hasNoteInShooter()).andThen(
          new ParallelCommandGroup(
            new SetTurretAngle(turret, 23.),
            // new AimAtSpeaker(turret, drivetrain),
            new InstantCommand(() -> {
              loader.stop();
              intake.stop();
            }, loader, intake)))
      ),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter())//,
      //END SHOOT OR TRANSFER THEN SHOOT ------
      // new ParallelDeadlineGroup(
      //   AutoBuilder.followPath(path5).until(() -> loader.hasNoteInShooter()),
      //   new RevShooter(drivetrain, shooter, loader).withTimeout(2),
      //   new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      // )
      // new ParallelDeadlineGroup(
      //   AutoBuilder.followPath(path6),
      //   new RevShooter(drivetrain, shooter, loader).withTimeout(2),
      //   new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()).andThen(new AimAtSpeaker(turret, drivetrain))
      // ),
      // //SHOOT OR TRANSFER THEN SHOOT ---------
      // new ConditionalCommand(
      //   new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.),
      //   new ParallelDeadlineGroup(
      //     new AutoTransfer(intakePivot, intake, elevator, turret, loader),
      //     new RevShooter(drivetrain, shooter, loader)).andThen(
      //       new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.)).unless(() -> !intake.hasNoteInIntake()), 
      //   () -> loader.hasNoteInShooter())
      // //END SHOOT OR TRANSFER THEN SHOOT ------
    );
  }

  public static Command Start_Source_End_15_14(Drivetrain drivetrain, IntakePivot intakePivot, Intake intake, Loader loader, Turret turret, Shooter shooter, Elevator elevator, LEDs leds) {
    PathPlannerPath path0 = PathPlannerPath.fromPathFile("SSource-EDownstage");
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("SDownstage-E15");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("S15-EDownstage");
    PathPlannerPath path3 = PathPlannerPath.fromPathFile("SDownstage-E14");
    PathPlannerPath path4 = PathPlannerPath.fromPathFile("S14-EDownstage");

    return Commands.sequence(
      new InstantCommand(
        () -> {
          if (DriverStation.getAlliance().get() == Alliance.Blue){
            drivetrain.setFieldPosition(new Pose2d(new Translation2d(1.48, 1.55), new Rotation2d(0.)));
          } else {
            drivetrain.setFieldPosition(new Pose2d(new Translation2d(kFieldX - 1.48, 1.55), new Rotation2d(Math.PI)));
          }
        }
        ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path0),
        new SetTurretAngle(turret, 23.),
        new RunCommand(() -> shooter.setShooterSpeed(5676.), shooter)),
      new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path1).until(() -> loader.hasNoteInShooter()),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      ),
      // new WaitCommand(0.3),
      // new AutoTransfer(intakePivot, intake, elevator, turret, loader).withTimeout(1.),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path2),
        new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        new WaitCommand(0.5).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).until(() -> !intake.hasNoteInIntake() && loader.hasNoteInShooter()).andThen(
          new ParallelCommandGroup(
            new SetTurretAngle(turret, 23.),
            new InstantCommand(() -> {
              loader.stop();
              intake.stop();
            }, loader, intake))))
      ),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter()),
      //END SHOOT OR TRANSFER THEN SHOOT ------
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path3).until(() -> loader.hasNoteInShooter()),
        new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      ),
      // new WaitCommand(0.3),
      // new AutoTransfer(intakePivot, intake, elevator, turret, loader).withTimeout(1.),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path4),
        new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        new WaitCommand(0.5).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).until(() -> !intake.hasNoteInIntake() && loader.hasNoteInShooter()).andThen(
          new ParallelCommandGroup(
            new SetTurretAngle(turret, 23.),
            new InstantCommand(() -> {
              loader.stop();
              intake.stop();
            }, loader, intake))))
      ),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter())//,
      //END SHOOT OR TRANSFER THEN SHOOT ------
      // new ParallelDeadlineGroup(
      //   AutoBuilder.followPath(path5).until(() -> loader.hasNoteInShooter()),
      //   new RevShooter(drivetrain, shooter, loader),
      //   new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      // )//,
      // new ParallelDeadlineGroup(
      //   AutoBuilder.followPath(path6),
      //   new RevShooter(drivetrain, shooter, loader).withTimeout(2),
      //   new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()).andThen(new AimAtSpeaker(turret, drivetrain))
      // ),
      // //SHOOT OR TRANSFER THEN SHOOT ---------
      // new ConditionalCommand(
      //   new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.),
      //   new ParallelDeadlineGroup(
      //     new AutoTransfer(intakePivot, intake, elevator, turret, loader),
      //     new RevShooter(drivetrain, shooter, loader)).andThen(
      //       new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.)).unless(() -> !intake.hasNoteInIntake()), 
      //   () -> loader.hasNoteInShooter())
      // //END SHOOT OR TRANSFER THEN SHOOT ------
    );
  }

  public static Command Start_Source_End_14_13(Drivetrain drivetrain, IntakePivot intakePivot, Intake intake, Loader loader, Turret turret, Shooter shooter, Elevator elevator, LEDs leds) {
    PathPlannerPath path0 = PathPlannerPath.fromPathFile("SSource-EDownstage");
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("SDownstage-E14");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("S14-EDownstage");
    PathPlannerPath path3 = PathPlannerPath.fromPathFile("SDownstage-E13");
    PathPlannerPath path4 = PathPlannerPath.fromPathFile("S13-EDownstage");

    return Commands.sequence(
      new InstantCommand(
        () -> {
          if (DriverStation.getAlliance().get() == Alliance.Blue){
            drivetrain.setFieldPosition(new Pose2d(new Translation2d(1.48, 1.55), new Rotation2d(0.)));
          } else {
            drivetrain.setFieldPosition(new Pose2d(new Translation2d(kFieldX - 1.48, 1.55), new Rotation2d(Math.PI)));
          }
        }
        ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path0),
        new SetTurretAngle(turret, 23.5),
        new RunCommand(() -> shooter.setShooterSpeed(5676.), shooter)),
      new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path1).until(() -> loader.hasNoteInShooter()),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      ),
      // new WaitCommand(0.3),
      // new AutoTransfer(intakePivot, intake, elevator, turret, loader).withTimeout(1.),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path2),
        new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        new WaitCommand(0.5).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).until(() -> !intake.hasNoteInIntake() && loader.hasNoteInShooter()).andThen(
          new ParallelCommandGroup(
            new SetTurretAngle(turret, 23.5),
            new InstantCommand(() -> {
              loader.stop();
              intake.stop();
            }, loader, intake))))
      ),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter()),
      //END SHOOT OR TRANSFER THEN SHOOT ------
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path3).until(() -> loader.hasNoteInShooter()),
        new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      ),
      // new WaitCommand(0.3),
      // new AutoTransfer(intakePivot, intake, elevator, turret, loader).withTimeout(1.),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path4),
        new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        new WaitCommand(0.5).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).until(() -> !intake.hasNoteInIntake() && loader.hasNoteInShooter()).andThen(
          new ParallelCommandGroup(
            new SetTurretAngle(turret, 23.5),
            new InstantCommand(() -> {
              loader.stop();
              intake.stop();
            }, loader, intake))))
      ),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter())//,
      //END SHOOT OR TRANSFER THEN SHOOT ------
      // new ParallelDeadlineGroup(
      //   AutoBuilder.followPath(path5).until(() -> loader.hasNoteInShooter()),
      //   new RevShooter(drivetrain, shooter, loader),
      //   new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      // )//,
      // new ParallelDeadlineGroup(
      //   AutoBuilder.followPath(path6),
      //   new RevShooter(drivetrain, shooter, loader).withTimeout(2),
      //   new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()).andThen(new AimAtSpeaker(turret, drivetrain))
      // ),
      // //SHOOT OR TRANSFER THEN SHOOT ---------
      // new ConditionalCommand(
      //   new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.),
      //   new ParallelDeadlineGroup(
      //     new AutoTransfer(intakePivot, intake, elevator, turret, loader),
      //     new RevShooter(drivetrain, shooter, loader)).andThen(
      //       new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.)).unless(() -> !intake.hasNoteInIntake()), 
      //   () -> loader.hasNoteInShooter())
      // //END SHOOT OR TRANSFER THEN SHOOT ------
    );
  }

  public static Command Start_3_End_2_1_12(Drivetrain drivetrain, IntakePivot intakePivot, Intake intake, Loader loader, Turret turret, Shooter shooter, Elevator elevator, LEDs leds) {
    
    PathPlannerPath path0 = PathPlannerPath.fromPathFile("PS3-IS3");
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("S3-E2");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("S2-E1");
    PathPlannerPath path3 = PathPlannerPath.fromPathFile("S1-E11");
    PathPlannerPath path4 = PathPlannerPath.fromPathFile("S11-EUpstage");

    return Commands.sequence(
      new InstantCommand(
        () -> {
          if (DriverStation.getAlliance().get() == Alliance.Blue){
            drivetrain.setFieldPosition(new Pose2d(new Translation2d(1.44, 4.54), new Rotation2d(0.)));
          } else {
            drivetrain.setFieldPosition(new Pose2d(new Translation2d(kFieldX - 1.44, 4.54), new Rotation2d(Math.PI)));
          }
        }
        ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path0),
        new SetTurretAngle(turret, 38.63),
        new RunCommand(() -> shooter.setShooterSpeed(5676.*0.5), shooter)),
      new ShootNoteAtSpeedAndAngle(shooter, turret, loader, 5676*.5, 38.63),
      // new ParallelDeadlineGroup(
      //   new AutoCollect(intakePivot, intake, turret, loader).withTimeout(1.5).until(() -> intake.hasNoteInIntake()),
      //   new DriveWithJoysticksRobotRelative(drivetrain, () -> 0., () -> -0.3,() -> 0.)  
      // ),
      // new AutoTransfer(intakePivot, intake, elevator, turret, loader).withTimeout(1.).until(() -> loader.hasNoteInShooter()),
      // new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))),
      new AutoCollect(intakePivot, intake, turret, loader).withTimeout(1.),
      new AutoTransfer(intakePivot, intake, elevator, turret, loader).withTimeout(1.),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      // new ConditionalCommand(
      //   new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
      //   new ParallelDeadlineGroup(
      //     new AutoTransfer(intakePivot, intake, elevator, turret, loader),
      //     new RevShooter(drivetrain, shooter, loader)).andThen(
      //       new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
      //   () -> loader.hasNoteInShooter()),
      //END SHOOT OR TRANSFER THEN SHOOT ------
      new ParallelCommandGroup(
        AutoBuilder.followPath(path1).until(() -> loader.hasNoteInShooter()),
        new ShootNoteAtSpeedAndAngle(shooter, turret, loader, 5676 * .5, 42.).until(() -> !loader.hasNoteInShooter())
        // new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        // new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      ),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      // new ConditionalCommand(
      //   new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
      //   new ParallelDeadlineGroup(
      //     new AutoTransfer(intakePivot, intake, elevator, turret, loader),
      //     new RevShooter(drivetrain, shooter, loader)).andThen(
      //       new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
      //   () -> loader.hasNoteInShooter()),
      //END SHOOT OR TRANSFER THEN SHOOT ------
      new AutoCollect(intakePivot, intake, turret, loader).withTimeout(1.),
      new AutoTransfer(intakePivot, intake, elevator, turret, loader).withTimeout(1.),
      new ParallelCommandGroup(
        AutoBuilder.followPath(path2),
        new ShootNoteAtSpeedAndAngle(shooter, turret, loader, 5676*.5, 37.)
        // new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake())),
        // new RevShooter(drivetrain, shooter, loader)
      ),
      new AutoCollect(intakePivot, intake, turret, loader).withTimeout(1.),
      new AutoTransfer(intakePivot, intake, elevator, turret, loader).withTimeout(1.),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      // new ConditionalCommand(
      //   new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
      //   new ParallelDeadlineGroup(
      //     new AutoTransfer(intakePivot, intake, elevator, turret, loader),
      //     new RevShooter(drivetrain, shooter, loader)).andThen(
      //       new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
      //   () -> loader.hasNoteInShooter()),
      //END SHOOT OR TRANSFER THEN SHOOT ------
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path3),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake())),
        new RevShooter(drivetrain, shooter, loader)
        ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path4),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake())),
        new RevShooter(drivetrain, shooter, loader)),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter())
      //END SHOOT OR TRANSFER THEN SHOOT ------
    );

  }

  public static Command Start_3_End_13_14_15(Drivetrain drivetrain, IntakePivot intakePivot, Intake intake, Loader loader, Turret turret, Shooter shooter, Elevator elevator, LEDs leds) {
    PathPlannerPath path0 = PathPlannerPath.fromPathFile(       "PS3-IS3");
    PathPlannerPath path1 = PathPlannerPath.fromPathFile(       "IS3-E3");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile(        "S3-E15");
    PathPlannerPath path3 = PathPlannerPath.fromPathFile(       "S15-EDownstage");
    PathPlannerPath path4 = PathPlannerPath.fromPathFile("SDownstage-E14");
    PathPlannerPath path5 = PathPlannerPath.fromPathFile(       "S14-EDownstage");
    PathPlannerPath path6 = PathPlannerPath.fromPathFile("SDownstage-E13");
    PathPlannerPath path7 = PathPlannerPath.fromPathFile(       "S13-EUpstage");

    return Commands.sequence(
      new InstantCommand(
        () -> {
          if (DriverStation.getAlliance().get() == Alliance.Blue){
            drivetrain.setFieldPosition(new Pose2d(new Translation2d(1.44, 4.54), new Rotation2d(0.)));
          } else {
            drivetrain.setFieldPosition(new Pose2d(new Translation2d(kFieldX - 1.44, 4.54), new Rotation2d(Math.PI)));
          }
        }
        ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path0),
        new RevShooter(drivetrain, shooter, loader)),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter()),
      //END SHOOT OR TRANSFER THEN SHOOT ------
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path1).until(() -> loader.hasNoteInShooter()),
        new RevShooter(drivetrain, shooter, loader),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      ),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter()),
      //END SHOOT OR TRANSFER THEN SHOOT ------
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path2).until(() -> loader.hasNoteInShooter()),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path3),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake())),
        new RevShooter(drivetrain, shooter, loader)
      ),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter()),
      //END SHOOT OR TRANSFER THEN SHOOT ------
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path4).until(() -> loader.hasNoteInShooter()),
        new AutoCollect(intakePivot, intake, turret, loader)
      ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path5),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake())),
        new RevShooter(drivetrain, shooter, loader)
      ),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter()),
      //END SHOOT OR TRANSFER THEN SHOOT ------
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path6).until(() -> loader.hasNoteInShooter()),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake()))
      ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path7),
        new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake())),
        new RevShooter(drivetrain, shooter, loader)
      ),
      new AutoCollect(intakePivot, intake, turret, loader).andThen(new AutoTransfer(intakePivot, intake, elevator, turret, loader).unless(() -> !intake.hasNoteInIntake())),
      //SHOOT OR TRANSFER THEN SHOOT ---------
      new ConditionalCommand(
        new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
        new ParallelDeadlineGroup(
          new AutoTransfer(intakePivot, intake, elevator, turret, loader),
          new RevShooter(drivetrain, shooter, loader)).andThen(
            new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)).unless(() -> !intake.hasNoteInIntake()), 
        () -> loader.hasNoteInShooter())
      //END SHOOT OR TRANSFER THEN SHOOT ------
    );
  }

  public static Command Start_1_End_Upstage(Drivetrain drivetrain, IntakePivot intakePivot, Intake intake, Loader loader, Turret turret, Shooter shooter, Elevator elevator, LEDs leds) {
    return Commands.sequence(
      new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds),
      new ParallelDeadlineGroup(
        new DriveWithJoysticksFieldRelative(drivetrain, () -> 0.4, () -> 0., () -> 0., () -> 0.),
        new AutoCollect(intakePivot, intake, turret, loader)),
      new AutoTransfer(intakePivot, intake, elevator, turret, loader),
      new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0., leds)
    );
  }

}
