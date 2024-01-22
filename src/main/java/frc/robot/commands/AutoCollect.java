// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCollect extends SequentialCommandGroup {
  /** Creates a new AutoCollect. */
  public AutoCollect(Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new RunCommand(() -> intake.setIntake(0.5), intake).withTimeout(0.1),
    new Collect(intake, 0.5),
    new RunCommand(() -> intake.setIntake(0.2), intake).until(() -> intake.hasNoteInIntake()),
    new WaitCommand(0.1),
    new InstantCommand(() -> intake.setIntake(0.), intake)
    );
  }
}
