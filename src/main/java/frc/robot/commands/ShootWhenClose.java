// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.Shooter.PID.*;
import static frc.robot.Constants.LEDs.kConfidentShotRange;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Swerve.PID.*;

public class ShootWhenClose extends Command {
  private Drivetrain m_drivetrain;
  private Shooter m_shooter;
  private Turret m_turret;
  private Loader m_loader;
  private PIDController angleController = new PIDController(kAngleP, kAngleI, kAngleD);
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
  private PIDController LLturnController = new PIDController(kLLTurnP, kLLTurnI, kLLTurnD);
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private DoubleSupplier xSpeed, ySpeed, m_precision;
  private Double X, Y, rot, m_precisionFactor, m_xSpeed, m_ySpeed;
  /** Creates a new ShootWhileMoving. */
  public ShootWhenClose(Drivetrain drivetrain, Shooter shooter, Turret turret, Loader loader, DoubleSupplier x_speed, DoubleSupplier y_speed, DoubleSupplier precision) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_loader = loader;
    m_turret = turret;
    m_precision = precision;
    xSpeed = x_speed;
    ySpeed = y_speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, drivetrain, loader, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.setTolerance(3);
    turnController.setSetpoint(0.0);
    turnController.enableContinuousInput(-180, 180);
    LLturnController.setSetpoint(0.);
    LLturnController.setTolerance(3);
    angleController.setSetpoint(0);
    angleController.setTolerance(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_precisionFactor = Math.pow(kDrivingPrecisionMultiplier, m_precision.getAsDouble());
    Y = MathUtil.applyDeadband(ySpeed.getAsDouble(), kDriveDeadband) * m_precisionFactor;
    X = MathUtil.applyDeadband( xSpeed.getAsDouble(), kDriveDeadband) * m_precisionFactor;
        // SmartDashboard.putNumber("xSpeed", X);
    var speedAdjustmentFactor = kMaxSpeedMetersPerSecond * kSpeedMultiplier;
    m_xSpeed = -m_xLimiter.calculate(Y * Y * Math.signum(Y) * speedAdjustmentFactor);
    
    m_ySpeed = -m_yLimiter.calculate(X * X * Math.signum(X) * speedAdjustmentFactor);

    var dis = m_drivetrain.getDistanceToSpeaker();
    var dDis = m_drivetrain.getChangeInDistanceToSpeaker(m_xSpeed, m_ySpeed);
    
    rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees() - (m_drivetrain.getAngleToSpeaker() /*+ kShootingRotationAdjustmentMultiplier * dTheta*/));

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      m_xSpeed = -m_xSpeed;
      m_ySpeed = -m_ySpeed;
    }

    var distanceMultiplier = dis/4.5;
    if (X+Y >= 0.6) {
      distanceMultiplier += 0.2;
    }
    distanceMultiplier = distanceMultiplier > 1 ? 1 : distanceMultiplier;
    distanceMultiplier = distanceMultiplier < 0.8 ? 0.8 : distanceMultiplier;
    
    var spinMultiplier = 0.75;
    // var spinMultiplier = 0.85 / distanceMultiplier;
    // spinMultiplier = spinMultiplier > 1 ? 1 : spinMultiplier;
    // spinMultiplier = 1.;

    m_shooter.setRightShooterSpeed(distanceMultiplier * 5676.0);
    m_shooter.setLeftShooterSpeed(distanceMultiplier * 5676.0 * spinMultiplier);

    m_turret.setAngleMotor(-(angleController.calculate((m_turret.getRequiredShooterAngleFromTable(dis) * 180 / Math.PI) - m_turret.getShooterAngle())));

    SmartDashboard.putNumber("Required Angle", m_turret.getRequiredShooterAngleFromTable(dis) * 180 / Math.PI);

    
    SmartDashboard.putBoolean("Angle?", angleController.atSetpoint());
    SmartDashboard.putBoolean("LLTurn?", LLturnController.atSetpoint());
    SmartDashboard.putBoolean("Turn?", turnController.atSetpoint());
    SmartDashboard.putBoolean("Speed?", m_shooter.getRightShooterSpeed() >= distanceMultiplier * 5676.0 * 0.8);
    
    m_drivetrain.drive(m_xSpeed, m_ySpeed, rot, true);
    // if (dis <= kConfidentShotRange) {
    //   m_drivetrain.drive(0., 0., rot, true);
    // } else {
    // }

    // if ((dis <= kConfidentShotRange) && angleController.atSetpoint() && m_shooter.getRightShooterSpeed() >= distanceMultiplier * 5676.0 * 0.8 && turnController.atSetpoint()) {
    //   m_loader.setLoaderMotor(1.);
    // } else {
    //   m_loader.stop();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setAngleMotor(0.0);
    m_loader.setLoaderMotor(0.0);
    m_shooter.setShooterSpeed(0.0);
    m_drivetrain.stop();
    angleController.close();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_loader.hasNoteInShooter();
  }
}
