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
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.Shooter.PID.*;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.SwerveModuleConstants.*;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class ShootWhileMoving extends Command {
  private Drivetrain m_drivetrain;
  private Shooter m_shooter;
  private PIDController angleController = new PIDController(kAngleP, kAngleI, kAngleD);
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private DoubleSupplier xSpeed, ySpeed, m_precision;
  private Double X, Y, rot, m_precisionFactor, m_xSpeed, m_ySpeed;
  /** Creates a new ShootWhileMoving. */
  public ShootWhileMoving(Drivetrain drivetrain, Shooter shooter, DoubleSupplier x_speed, DoubleSupplier y_speed, DoubleSupplier precision) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_precision = precision;
    xSpeed = x_speed;
    ySpeed = y_speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_precisionFactor = Math.pow(0.4, m_precision.getAsDouble());
    X = xSpeed.getAsDouble();
    Y = ySpeed.getAsDouble();
    X = X > kAutoDriveSpeedLimiter ? kAutoDriveSpeedLimiter : X;
    X = X < -kAutoDriveSpeedLimiter ? -kAutoDriveSpeedLimiter : X;
    Y = Y > kAutoDriveSpeedLimiter ? kAutoDriveSpeedLimiter : Y;
    Y = Y < -kAutoDriveSpeedLimiter ? -kAutoDriveSpeedLimiter : Y;
    turnController.setTolerance(5);
    turnController.setSetpoint(0.0);
    turnController.enableContinuousInput(-180, 180);

    angleController.setSetpoint(0);
    angleController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // SmartDashboard.putNumber("xSpeed", X);
    m_xSpeed =
    -m_xLimiter.calculate(MathUtil.applyDeadband(Math.pow(Y, 2) * Math.signum(Y), kDriveDeadband))
    * kMaxSpeedMetersPerSecond * kSpeedMultiplier * m_precisionFactor;
    // SmartDashboard.putNumber("xSpeed after limiting", Y);
    
    m_ySpeed =
    -m_yLimiter.calculate(MathUtil.applyDeadband(Math.pow(X, 2) * Math.signum(X), kDriveDeadband))
    * kMaxSpeedMetersPerSecond * kSpeedMultiplier * m_precisionFactor;

    var dis = m_drivetrain.getDistanceToSpeaker();
    var dDis = m_drivetrain.getChangeInDistanceToSpeaker(m_xSpeed, m_ySpeed);
    
    var dTheta = -m_drivetrain.getChangeInAngleToSpeaker(m_xSpeed, m_ySpeed);
    dTheta = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? -dTheta : dTheta;
  
    // SmartDashboard.putNumber("Angle", m_drivetrain.getAngleToSpeaker());
    // SmartDashboard.putNumber("dTheta", dTheta);
    
    rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees() - m_drivetrain.getAngleToSpeaker() + kShootingAdjustmentMultiplier * dTheta);

    SmartDashboard.putNumber("ShooterAngle", m_shooter.getRequiredShooterAngle(dis, dDis));

    if (DriverStation.getAlliance().get() == Alliance.Red) {
      m_xSpeed = -m_xSpeed;
      m_ySpeed = -m_ySpeed;
    }


    m_shooter.setShooterSpeed(5676.0);

    // SmartDashboard.putNumber("AnglePID", angleController.calculate((m_shooter.getRequiredShooterAngle(dis, dDis)*180 / Math.PI) - m_shooter.getShooterAngle()));
    // SmartDashboard.putNumber("required angle", m_shooter.getRequiredShooterAngle(dis, dDis)*180 / Math.PI);


    m_shooter.setAngleMotor(angleController.calculate((m_shooter.getRequiredShooterAngle(dis, dDis)*180 / Math.PI) - m_shooter.getShooterAngle()));

    m_drivetrain.drive(m_xSpeed, m_ySpeed, rot, true);

    SmartDashboard.putBoolean("Angle?", angleController.atSetpoint());
    SmartDashboard.putBoolean("Turn?", turnController.atSetpoint());
    SmartDashboard.putBoolean("Speed?", m_shooter.getBottomShooterSpeed() >= 5676 * 0.9);

    if (angleController.atSetpoint() && m_shooter.getBottomShooterSpeed() >= 5676 * 0.9 && turnController.atSetpoint()) {
      m_shooter.setLoaderMotor(0.8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setAngleMotor(0.0);
    m_shooter.setLoaderMotor(0.0);
    m_shooter.setShooterSpeed(0.0);
    m_drivetrain.stop();
    angleController.close();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
