// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class RevShooter extends Command {
  /** Creates a new RevShooter. */
  private Drivetrain m_drivetrain;
  private Shooter m_shooter;
  private Loader m_loader;
  public RevShooter(Drivetrain drivetrain, Shooter shooter, Loader loader) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_loader = loader;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var dis = m_drivetrain.getDistanceToSpeaker();
    var distanceMultiplier = dis/5;
    distanceMultiplier = distanceMultiplier > 1 ? 1 : distanceMultiplier;
    distanceMultiplier = distanceMultiplier < 0.5 ? 0.5 : distanceMultiplier;
    if (!m_loader.hasNoteInShooter() || dis >= 8) {
      distanceMultiplier = 0.;
    }
    m_shooter.setRightShooterSpeed(distanceMultiplier * 5676.0);
    m_shooter.setLeftShooterSpeed(distanceMultiplier * 5676.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
