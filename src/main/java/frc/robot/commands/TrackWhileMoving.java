// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.SwerveModuleConstants.*;
import static frc.robot.Constants.Shooter.*;


public class TrackWhileMoving extends Command {
  private Drivetrain m_drivetrain;
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private DoubleSupplier xSpeed, ySpeed, m_precision;
  private Double X, Y, rot, m_precisionFactor, m_xSpeed, m_ySpeed;

  /** Creates a new TrackWhileMoving. */
  public TrackWhileMoving(Drivetrain drivetrain, Shooter shooter, DoubleSupplier x_speed, DoubleSupplier y_speed, DoubleSupplier precision) {
    m_drivetrain = drivetrain;
    m_precision = precision;
    xSpeed = x_speed;
    // SmartDashboard.putNumber("x input", x_speed.getAsDouble());
    // SmartDashboard.putNumber("Precision", precision.getAsDouble());
    ySpeed = y_speed;
    addRequirements(drivetrain, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
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
    turnController.setTolerance(1);
    turnController.setSetpoint(0.0);
    turnController.enableContinuousInput(-180, 180);
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

    // var dis = m_drivetrain.getDistanceToSpeaker();
    // var dDis = m_drivetrain.getChangeInDistanceToSpeaker(m_xSpeed, m_ySpeed);
    
    var dTheta = -m_drivetrain.getChangeInAngleToSpeaker(m_xSpeed, m_ySpeed);
    dTheta = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? -dTheta : dTheta;
  
    // SmartDashboard.putNumber("Angle", m_drivetrain.getAngleToSpeaker());
    // SmartDashboard.putNumber("dTheta", dTheta);
    
    rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees() - m_drivetrain.getAngleToSpeaker() + kShootingAdjustmentMultiplier * dTheta);
    
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      m_xSpeed = -m_xSpeed;
      m_ySpeed = -m_ySpeed;
    }

    m_drivetrain.drive(m_xSpeed, m_ySpeed, rot, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
