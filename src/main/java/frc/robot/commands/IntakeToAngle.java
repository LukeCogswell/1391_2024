// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

import static frc.robot.Constants.Intake.PID.*;
import static frc.robot.Constants.Intake.*;

public class IntakeToAngle extends Command {
  private IntakePivot m_intakePivot;
  private Double angle;
  private PIDController rotController = new PIDController(kIAngleP, kIAngleI, kIAngleD);
  /** Creates a new IntakeToAngle. */
  public IntakeToAngle(IntakePivot intakePivot, Double toAngle) {
    m_intakePivot = intakePivot;
    angle = toAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotController.setSetpoint(angle);
    rotController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var pwr = -rotController.calculate(m_intakePivot.getIntakeAngle());
    pwr = MathUtil.clamp(pwr, kMaxDownPower, kMaxUpPower);
    SmartDashboard.putNumber("?Power", pwr);
    m_intakePivot.setAngleMotor(pwr);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotController.close();
    m_intakePivot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return angleController.atSetpoint();
  }
}
