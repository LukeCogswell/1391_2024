// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Swerve.kAccelerationSeconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;

public class AutoTrackNote extends Command {
  private Drivetrain m_drivetrain;
  private Intake m_intake;
  private IntakePivot m_intakePivot;
  private PIDController rotController = new PIDController(0.05, 0.01, 0.0);
  private SlewRateLimiter driveLimiter = new SlewRateLimiter(4/kAccelerationSeconds);
  private Double drive, rot, ty;
  /** Creates a new AutoTrackNote. */

  public AutoTrackNote(Drivetrain drivetrain, Intake intake, IntakePivot intakePivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_intake = intake;
    m_intakePivot = intakePivot;
    addRequirements(drivetrain, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotController.setSetpoint(0.);
    rotController.setTolerance(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getTV()) {
      ty = (m_intake.getTY() + 25.);
      drive = ty/30.;
      drive = drive > .3 ? .3 : drive;
      rot = rotController.calculate(m_intake.getTX());
      rot = MathUtil.clamp(rot, -.3, .3);
      if (ty < 20) {
        m_intake.setIntake(0.6);
      }
      if (m_intakePivot.getIntakeAngle() <= 255) {
        drive = 0.;
      }
      drive = driveLimiter.calculate(drive);
      m_drivetrain.drive(drive, 0., rot, false);
    } else {
      m_intake.setIntake(m_intake.hasNoteInIntake() ? 0. : 0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.hasNoteInIntake();
  }
}
