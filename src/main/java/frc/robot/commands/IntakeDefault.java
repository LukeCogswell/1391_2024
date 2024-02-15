// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Intake.PID.*;

public class IntakeDefault extends Command {
  private Intake m_intake;
  private PIDController angleController = new PIDController(kIAngleP, kIAngleI, kIAngleD);
  /** Creates a new IntakeDefault. */
  public IntakeDefault(Intake intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.isDown()) {
      angleController.setSetpoint(kMinRotation);
    } else {
      angleController.setSetpoint(kMaxRotation);
    }

    if (!m_intake.isUp()) {
      m_intake.setAngleMotor(angleController.calculate(m_intake.getIntakeAngle()));
    } else {
      m_intake.setAngleMotor(0.);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleController.close();
    m_intake.setAngleMotor(0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
