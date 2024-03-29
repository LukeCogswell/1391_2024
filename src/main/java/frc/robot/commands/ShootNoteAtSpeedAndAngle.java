// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.Shooter.PID.*;

public class ShootNoteAtSpeedAndAngle extends Command {
  /** Creates a new ShootNote. */
  private Shooter m_shooter;
  private Turret m_turret;
  private Loader m_loader;
  private Double shotSpeed, angle;
  private PIDController angleController = new PIDController(kAngleP, kAngleI, kAngleD);

  public ShootNoteAtSpeedAndAngle(Shooter shooter, Turret turret, Loader loader, Double speed, Double Angle) {
    m_shooter = shooter;
    m_turret = turret;
    m_loader = loader;
    shotSpeed = speed;
    angle = Angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setSetpoint(angle);
    angleController.setTolerance(1.);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setShooterSpeed(shotSpeed);

    m_turret.setAngleMotor(-angleController.calculate(m_turret.getShooterAngle()));

    if (m_shooter.getLeftShooterSpeed() >= 0.9 * shotSpeed && angleController.atSetpoint()) {
        m_loader.setLoaderMotor(0.7);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_loader.setLoaderMotor(0.0);
    angleController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
