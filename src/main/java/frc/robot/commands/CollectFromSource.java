// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Elevator.kMinHeight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.PathfindingPoints.Red;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.PathfindingPoints.Blue;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectFromSource extends ParallelCommandGroup {
  /** Creates a new CollectFromSource. */
  public CollectFromSource(Drivetrain drivetrain, Turret turret, Shooter shooter, Loader loader, Elevator elevator, CommandXboxController driverController) {
    Pose2d sourcePoint = Blue.Source;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      sourcePoint = Red.Source;
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToPoint(drivetrain, sourcePoint).until(() -> loader.hasNoteInShooter()).andThen(
        new DriveWithJoysticksFieldRelative(drivetrain, () -> driverController.getLeftX(), () -> driverController.getLeftY(), () -> driverController.getRightX(), () -> driverController.getRightTriggerAxis())
      ),
      new SequentialCommandGroup(
        new WaitCommand(0.2),
        new ElevatorToHeight(elevator, kMinHeight)
        ),
      new SequentialCommandGroup(
        new RunCommand(() -> {
          shooter.setLeftShooterSpeed(-2000.);
          shooter.setRightShooterSpeed(-1000.);
          loader.setLoaderMotor(-0.2);
        }, loader, shooter).until(() -> loader.hasNoteInShooter()), 
        new WaitUntilCommand(() -> !loader.hasNoteInShooter()), 
        new InstantCommand(() -> shooter.stopShooter()),
        new RunCommand(() -> loader.setLoaderMotor(0.2), loader).until(() -> loader.hasNoteInShooter()),
        new InstantCommand(() -> loader.stop())
      ),
      new SetTurretAngle(turret, 60.)
    );
  }
}
