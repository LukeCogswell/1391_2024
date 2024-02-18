// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Intake.kMaxRotation;
import static frc.robot.Constants.Intake.kMinRotation;
import static frc.robot.Constants.Shooter.kTransferAngle;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCollect extends SequentialCommandGroup {
  /** Creates a new AutoCollect. */
  public AutoCollect(IntakePivot intakePivot, Intake intake, Turret turret, Loader loader) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ParallelRaceGroup(
      new SetTurretAngle(turret, kTransferAngle),
      new SequentialCommandGroup(
        // new RunCommand(() -> intake.setIntake(0.7), intake).withTimeout(0.1),
        // new Collect(intake, 0.7),
        // new RunCommand(() -> intake.setIntake(0.4), intake).until(() -> intake.hasNoteInIntake()),
        new InstantCommand(() -> {
          intake.setIntake(0.4);
        }, intake),
        new IntakeToAngle(intakePivot, kMinRotation).until(() -> intake.hasNoteInIntake()),
        new InstantCommand(() -> intake.stop(), intake),
        new IntakeToAngle(intakePivot, kMaxRotation).until(() -> intakePivot.isUp()),
        new InstantCommand(() -> {
          intake.setIntake(0.2);
          loader.setLoaderMotor(0.8);
        }, intake, loader),
        new WaitUntilCommand(() -> (loader.hasNoteInShooter())),
        // new WaitCommand(0.2),
        new InstantCommand(() -> {
          loader.setLoaderMotor(0.);
          intake.setIntake(0.);}, loader, intake)
        // new WaitCommand(0.1),
    )));
  }

}
