// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Elevator.kMinHeight;
import static frc.robot.Constants.Intake.kMaxRotation;
import static frc.robot.Constants.Shooter.kTransferAngle;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTransfer extends ParallelDeadlineGroup {
  /** Creates a new Transfer. */
  public AutoTransfer(IntakePivot intakePivot, Intake intake, Elevator elevator, Turret turret, Loader loader) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new SequentialCommandGroup(
        new ElevatorToHeight(elevator, 1.), //0.13
        new WaitUntilCommand(() -> ((intakePivot.getIntakeAngle() <= 189) && (elevator.getElevatorHeightL() >= 0.7))),
        new RunCommand(() -> {
          loader.setLoaderMotor(.6);
          intake.setIntake(.4);
        }, intake, loader).until(() -> (loader.hasNoteInShooter())),
        new InstantCommand(() -> {
          intake.setIntake(0.);
          loader.setLoaderMotor(0.);
        }, intake, loader),
        new ElevatorToHeight(elevator, kMinHeight)
      )
    );
    addCommands(
      new SetTurretAngle(turret, kTransferAngle),
      new IntakeToAngle(intakePivot, kMaxRotation).andThen(new IntakePivotDefault(intakePivot))
    );
  }
}
