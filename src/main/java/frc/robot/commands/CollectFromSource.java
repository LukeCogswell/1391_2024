// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectFromSource extends ParallelCommandGroup {
  /** Creates a new CollectFromAmp. */
  public CollectFromSource(Loader loader, Shooter shooter, Turret turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new SetTurretAngle(turret, 60.),
    new SequentialCommandGroup(  
    new RunCommand(() -> {
      shooter.setLeftShooterSpeed(-3000.);
      shooter.setRightShooterSpeed(-1000.);
      loader.setLoaderMotor(-0.2);
    }, loader, shooter).until(() -> loader.hasNoteInShooter()).andThen(new WaitUntilCommand(() -> !loader.hasNoteInShooter()).andThen(new InstantCommand(() -> {
      shooter.stopShooter();
      loader.stop();
    })))));
  }
}
