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
  private Double angle, multiplier, MAXDownPWR;
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
    if (angle <= 200.) {
      rotController.setPID(0.01, 0., 0.);
      multiplier = 1.;
    } else {
      multiplier = 1.;
    }
    MAXDownPWR = kMaxDownPower;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var ang = m_intakePivot.getIntakeAngle();
    var pwr = -rotController.calculate(ang);

    if (ang >= 180) {
      MAXDownPWR = 0.4;
    }
    if (ang >= 185) {
      MAXDownPWR = 0.4;
    }
    if (ang >= 210) {
      MAXDownPWR = 0.2;
    }
    if (ang >= 240.) {
    MAXDownPWR = 0.1;
    }      
    SmartDashboard.putNumber("MAXDOWN", MAXDownPWR);
    pwr = MathUtil.clamp(pwr, -MAXDownPWR, kMaxUpPower);
    // if (m_intakePivot.getIntakeAngle() <= 200 && angle <=200) {
    //   pwr = MathUtil.clamp(pwr, 0., 0.2);
    // }
    SmartDashboard.putNumber("?Power", pwr);
    m_intakePivot.setAngleMotor(pwr * multiplier);
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
