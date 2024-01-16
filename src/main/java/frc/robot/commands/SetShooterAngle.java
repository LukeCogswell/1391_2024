// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.Shooter.PID.*;

public class SetShooterAngle extends Command {
  private Shooter m_shooter;
  private Double m_angle;
  private PIDController angleController;
  /** Creates a new SetShooterAngle. */
  public SetShooterAngle(Shooter shooter, Double angle) {
    m_shooter = shooter;
    m_angle = angle;

    addRequirements(shooter);
    
    angleController = new PIDController(kAngleP, kAngleI, kAngleD);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setSetpoint(m_angle);
    angleController.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setAngleMotor(angleController.calculate(m_shooter.getShooterAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setAngleMotor(0.0);
    angleController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleController.atSetpoint();
  }
}
