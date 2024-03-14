// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import static frc.robot.Constants.Shooter.PID.*;

public class SetTurretAngle extends Command {
  private Turret m_turret;
  private Double m_angle;
  private PIDController angleController;
  /** Creates a new SetShooterAngle. */
  public SetTurretAngle(Turret turret, Double angle) {
    m_turret = turret;
    m_angle = angle;

    addRequirements(turret);
    
    angleController = new PIDController(kAngleP, kAngleI, kAngleD);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setSetpoint(m_angle);
    angleController.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // angleController.setPID(
    //   SmartDashboard.getEntry("P").getDouble(kAngleP),
    //   SmartDashboard.getEntry("I").getDouble(kAngleI),
    //   SmartDashboard.getEntry("D").getDouble(kAngleD));
    m_turret.setAngleMotor(MathUtil.clamp(angleController.calculate(m_turret.getShooterAngle()), -1, 1));
    SmartDashboard.putBoolean("Angled?", angleController.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setAngleMotor(0.0);
    angleController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return angleController.atSetpoint();
  }
}
