// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Intake.PID.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Collect extends Command {
  private Intake m_intake;
  private Double runSpeed;
  private PIDController angleController = new PIDController(kIAngleP, kIAngleI, kIAngleD);
  /** Creates a new Collect. */
  public Collect(Intake intake, Double speed) {
    m_intake = intake;
    runSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntake(runSpeed);
    angleController.setSetpoint(kMinRotation);
    angleController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.hasNoteInIntake()) {
      m_intake.setIntake(0.);
      angleController.setSetpoint(kMaxRotation);
    }
    m_intake.setAngleMotor(angleController.calculate(m_intake.getIntakeAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntake(0.);
    m_intake.setAngleMotor(0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.hasNoteInIntake() && m_intake.isUp();
    // return false;
  }
}
