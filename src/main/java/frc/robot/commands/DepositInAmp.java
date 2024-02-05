// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DepositInAmp extends SequentialCommandGroup {
  private Pose2d ampPoint = Constants.PathfindingPoints.Red.Amp;
  /** Creates a new DepositInAmp. */
  public DepositInAmp(Drivetrain drivetrain, Intake intake, Loader loader, Turret turret, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new InstantCommand(() -> {
        if (DriverStation.getAlliance().isPresent()) {
          if (DriverStation.getAlliance().get() == Alliance.Red) {
            ampPoint = Constants.PathfindingPoints.Red.Amp;
          } else {
            ampPoint = Constants.PathfindingPoints.Blue.Amp;
          }
        }
      }),
      new ParallelCommandGroup(
        new AutoTransfer(intake, shooter, turret, loader).until(() -> loader.hasNoteInShooter()),
        drivetrain.getCommandToPathfindToPoint(ampPoint, 0.).until(() -> drivetrain.getTID() == 5 || drivetrain.getTID() == 6)),
      new SequentialCommandGroup(
        new AlignWithAprilTag(drivetrain, 1.1),
        new DriveForDistanceInDirection(drivetrain, 0., 0.5),
        new InstantCommand(() -> shooter.setShooterSpeed(1000.), shooter),
        new WaitCommand(0.4),
        new InstantCommand(() -> {
          loader.setLoaderMotor(0.8);
          shooter.setShooterSpeed(1000.);
          }),
        new WaitCommand(0.4),
        new InstantCommand(() -> {
          loader.stop();
          shooter.stopShooter();
        }, shooter, loader)
      )
    );
  }
}
