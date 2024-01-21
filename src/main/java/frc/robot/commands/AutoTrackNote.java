// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class AutoTrackNote extends Command {
  private Drivetrain m_drivetrain;
  private Intake m_intake;
  private PIDController rotController = new PIDController(0.075, 0.01, 0.0);
  private Double drive, rot, ty;
  /** Creates a new AutoTrackNote. */

  public AutoTrackNote(Drivetrain drivetrain, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_intake = intake;
    addRequirements(drivetrain, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotController.setSetpoint(0.);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getTV()) {
      ty = (m_intake.getTY() + 30);
      drive = ty/20;
      drive = drive > 2 ? 2 : drive;
      rot = rotController.calculate(m_intake.getTX());
      rot = rot > 0.5 ? 0.5 : rot;
      if (ty < 20) {
        m_intake.setIntake(m_intake.currentHasNoteInIntake() ? 0.3 : 0.6);
      }
      m_drivetrain.drive(drive, 0., rot, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
    m_intake.setIntake(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.hasNoteInIntake();
  }
}
