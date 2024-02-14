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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

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

  public static Command Start_3_End_13_14_15(Drivetrain drivetrain, Intake intake, Loader loader, Turret turret, Shooter shooter) {
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
      new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.),
      new ParallelCommandGroup(
        AutoBuilder.followPath(path1).until(() -> loader.hasNoteInShooter()),
        new RevShooter(drivetrain, shooter, loader).withTimeout(2),
        new AutoCollect(intake, turret, loader).withTimeout(2)
      ),
      new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path2).until(() -> loader.hasNoteInShooter()),
        new AutoCollect(intake, turret, loader)
      ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path3),
        new AutoCollect(intake, turret, loader),
        new RevShooter(drivetrain, shooter, loader)
      ),
      new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path4).until(() -> loader.hasNoteInShooter()),
        new AutoCollect(intake, turret, loader)
      ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path5),
        new AutoCollect(intake, turret, loader),
        new RevShooter(drivetrain, shooter, loader)
      ),
      new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path6).until(() -> loader.hasNoteInShooter()),
        new AutoCollect(intake, turret, loader)
      ),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path7),
        new AutoCollect(intake, turret, loader),
        new RevShooter(drivetrain, shooter, loader)
      ),
      new AutoCollect(intake, turret, loader).until(() -> loader.hasNoteInShooter()),
      new ShootWhileMoving(drivetrain, shooter, turret, loader, () -> 0., () -> 0., () -> 0.)
    );
  }

}
