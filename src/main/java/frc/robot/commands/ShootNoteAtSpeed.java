// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;

public class ShootNoteAtSpeed extends Command {
  /** Creates a new ShootNote. */
  private Shooter m_shooter;
  private Loader m_loader;
  private Double shotSpeed;
  private Trigger shootingTrigger, incrementPowerTrigger, incrementPowerUpTrigger;
  private Boolean adj;

  public ShootNoteAtSpeed(Shooter shooter, Loader loader, Double speed, Trigger shootTrigger, Trigger incrementPower, Trigger incrementUPP, Boolean adjustable) {
    m_shooter = shooter;
    m_loader = loader;
    shotSpeed = speed;
    shootingTrigger = shootTrigger;
    incrementPowerTrigger = incrementPower;
    incrementPowerUpTrigger = incrementUPP;
    adj = adjustable;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("Shot Speed", shotSpeed);
    // bottomShooterController.setSetpoint(shotSpeed);
    // topShooterController.setSetpoint(shotSpeed);
    // topShooterController.setTolerance(100);
    // bottomShooterController.setTolerance(100);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (adj) {
      shotSpeed = SmartDashboard.getEntry("Shot Speed").getDouble(shotSpeed);
    }
    if (incrementPowerTrigger.getAsBoolean()) {
      shotSpeed = shotSpeed - 300;
    }
    if (incrementPowerUpTrigger.getAsBoolean()) {
      shotSpeed = shotSpeed + 300;
    }

    m_shooter.setShooterSpeed(shotSpeed);
    // var botMotSpeed = bottomShooterController.calculate(m_shooter.getBottomShooterSpeed());
    // var topMotSpeed = topShooterController.calculate(m_shooter.getTopShooterSpeed());
    
    // SmartDashboard.putNumber("Top Speed", topMotSpeed);
    
    // m_shooter.setBottomShooterSpeed(botMotSpeed);
    // m_shooter.setTopShooterSpeed(topMotSpeed);

    // if (bottomShooterController.atSetpoint() && topShooterController.atSetpoint()) {
    //   m_shooter.setLoaderMotor(0.4);
    // }
    if (shootingTrigger.getAsBoolean()) {
        m_loader.setLoaderMotor(0.7);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_loader.setLoaderMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
