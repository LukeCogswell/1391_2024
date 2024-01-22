// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTransfer extends ParallelRaceGroup {
  /** Creates a new Transfer. */
  public AutoTransfer(Intake intake, Shooter shooter, Turret turret, Loader loader) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetTurretAngle(turret, 25.),
      new SequentialCommandGroup(
        new InstantCommand(() -> {
          loader.setLoaderMotor(0.5);
          intake.setIntake(0.2);
        }, loader, intake),
        new WaitUntilCommand(() -> !intake.hasNoteInIntake()),
        new InstantCommand(() -> {
          intake.setIntake(0.0);
          loader.setLoaderMotor(-0.5);
        }, loader, intake),
        new WaitCommand(0.06),
        new InstantCommand(() -> loader.setLoaderMotor(0.), loader)
      )
    );
  }
}
