// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Intake.PID.*;

public class IntakePivotDefault extends Command {
  private IntakePivot m_intakePivot;
  private PIDController angleController = new PIDController(kIAngleP, kIAngleI, kIAngleD);
  /** Creates a new IntakeDefault. */
  public IntakePivotDefault(IntakePivot intakePivot) {
    m_intakePivot = intakePivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intakePivot.isDown()) {
      angleController.setSetpoint(kMinRotation);
    } else {
      angleController.setSetpoint(kMaxRotation);
    }

    SmartDashboard.putNumber("SETPOINT", angleController.getSetpoint());

    if (!m_intakePivot.isUp()) {
      var pwr = angleController.calculate(m_intakePivot.getIntakeAngle());
      SmartDashboard.putNumber("PrePower", pwr);
      pwr = MathUtil.clamp(pwr, kMaxDownPower, kMaxUpPower);
      SmartDashboard.putNumber("MidPower", pwr);
      m_intakePivot.setAngleMotor(pwr);
    } else {
      m_intakePivot.setAngleMotor(0.);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleController.close();
    m_intakePivot.setAngleMotor(0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
