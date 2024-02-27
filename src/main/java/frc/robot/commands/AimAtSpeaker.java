// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Shooter.PID.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class AimAtSpeaker extends Command {
  private Turret m_turret;
  private Drivetrain m_drivetrain;
  private PIDController angleController = new PIDController(kAngleP, kAngleI, kAngleD);
  /** Creates a new AimAtSpeaker. */
  public AimAtSpeaker(Turret turret, Drivetrain drivetrain) {
    m_turret = turret;
    m_drivetrain = drivetrain;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setSetpoint(0.);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var dis = m_drivetrain.getDistanceToSpeaker();
    m_turret.setAngleMotor(-(angleController.calculate((m_turret.getRequiredShooterAngleFromTable(dis, 0.) * 180 / Math.PI) - m_turret.getShooterAngle())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
