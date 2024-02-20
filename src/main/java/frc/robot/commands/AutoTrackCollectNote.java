// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Intake.kMinRotation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTrackCollectNote extends SequentialCommandGroup {
  /** Creates a new AutoTrackCollectNote. */
  public AutoTrackCollectNote(Elevator elevator, Drivetrain drivetrain, Intake intake, IntakePivot intakePivot, Loader loader, Turret turret, Shooter shooter, CommandXboxController driverController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new IntakeToAngle(intakePivot, kMinRotation).until(() -> intakePivot.isDown()),
        new RunCommand(() -> intake.setIntake(-0.7), intake).withTimeout(0.4)),
      new AutoTrackNote(drivetrain, intake),
      new ParallelRaceGroup(
        new DriveWithJoysticksFieldRelative(
          drivetrain, 
          () -> driverController.getLeftX(), 
          () -> driverController.getLeftY(), 
          () -> driverController.getRightX(), 
          () -> driverController.getRightTriggerAxis() 
          ),
        new SequentialCommandGroup(
          new WaitCommand(0.2),
          new RunCommand(() -> intake.setIntake(0.6), intake).until(() -> intake.hasNoteInIntake()),
          new InstantCommand(() -> intake.setIntake(0.0), intake),
          new AutoTransfer(intakePivot, intake, elevator, turret, loader)
        )
      )
    );
  }
}
