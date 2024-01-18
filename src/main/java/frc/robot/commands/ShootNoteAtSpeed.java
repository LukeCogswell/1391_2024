// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.Shooter.PID.*;

public class ShootNoteAtSpeed extends Command {
  /** Creates a new ShootNote. */
  private Shooter m_shooter;
  private Double shotSpeed;
  private PIDController bottomShooterController, topShooterController;

  public ShootNoteAtSpeed(Shooter shooter, Double speed) {
    m_shooter = shooter;
    shotSpeed = speed;
    bottomShooterController = new PIDController(kP, kI, kD);
    topShooterController = new PIDController(kP, kI, kD);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bottomShooterController.setSetpoint(shotSpeed);
    topShooterController.setSetpoint(shotSpeed);
    topShooterController.setTolerance(100);
    bottomShooterController.setTolerance(100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var botMotSpeed = bottomShooterController.calculate(m_shooter.getBottomShooterSpeed());
    var topMotSpeed = topShooterController.calculate(m_shooter.getTopShooterSpeed());

    m_shooter.setBottomShooterSpeed(botMotSpeed/5676);
    m_shooter.setTopShooterSpeed(topMotSpeed/5676);

    if (bottomShooterController.atSetpoint() && topShooterController.atSetpoint()) {
      m_shooter.setLoaderMotor(0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_shooter.setLoaderMotor(0.0);
    topShooterController.close();
    bottomShooterController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
