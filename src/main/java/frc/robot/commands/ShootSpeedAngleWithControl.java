// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.Shooter.PID.*;

public class ShootSpeedAngleWithControl extends Command {
  /** Creates a new ShootNote. */
  private Shooter m_shooter;
  private Turret m_turret;
  private Loader m_loader;
  private Double shotSpeed, angle;
  private PIDController angleController = new PIDController(kAngleP, kAngleI, kAngleD);
  private Trigger shotTrigger;
  private Boolean adj;

  public ShootSpeedAngleWithControl(Shooter shooter, Turret turret, Loader loader, Double speed, Double Angle, Trigger shootingTrigger, Boolean adjustable) {
    m_shooter = shooter;
    m_turret = turret;
    m_loader = loader;
    shotSpeed = speed;
    angle = Angle;
    shotTrigger = shootingTrigger;
    adj = adjustable;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, loader, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setSetpoint(angle);
    angleController.setTolerance(.5);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (adj) {
      shotSpeed = SmartDashboard.getEntry("Shot Speed").getDouble(shotSpeed);
    }
    // m_shooter.setShooterSpeed(shotSpeed);
    m_shooter.setLeftShooterSpeed(shotSpeed*0.85);
    m_shooter.setRightShooterSpeed(shotSpeed);


    // if (!angleController.atSetpoint()) {
    m_turret.setAngleMotor(MathUtil.clamp(angleController.calculate(m_turret.getShooterAngle()), -0.4, 0.4));
    // } else {
    //   m_turret.setAngleMotor(0.);
    // }

    SmartDashboard.putBoolean("ShotAngle", angleController.atSetpoint());
    SmartDashboard.putBoolean("ShotSpeed", m_shooter.getRightShooterSpeed() >= 0.8 * shotSpeed);

    m_loader.setLoaderMotor(shotTrigger.getAsBoolean() ? 1. : 0.);

    // if (m_shooter.getRightShooterSpeed() >= 0.8 * shotSpeed && angleController.atSetpoint()) {
    //     m_loader.setLoaderMotor(1.);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_loader.setLoaderMotor(0.);
    m_turret.setAngleMotor(0.);
    angleController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !m_loader.hasNoteInShooter();
    return false;
  }
}
