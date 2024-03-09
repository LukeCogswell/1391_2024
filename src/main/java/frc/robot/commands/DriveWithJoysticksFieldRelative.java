// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Swerve.PID.*;
import static frc.robot.Constants.MeasurementConstants.*;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveWithJoysticksFieldRelative extends Command {

  Drivetrain m_drivetrain;

  DoubleSupplier m_x, m_y, m_theta, m_precision;
  Trigger m_faceForwards, fieldRelative;
  Rotation2d offsetDirection;

  boolean isRed;

  private PIDController turnController = new PIDController(.01, kTurnI, 0.002);
  
  double m_toAngle, m_xSpeed, m_ySpeed, m_thetaSpeed;

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(kDriveSlewRateLimit);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(kDriveSlewRateLimit);
  private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(kthetaSlewRateLimit);
  /** Creates a new Drive. */
  public DriveWithJoysticksFieldRelative(
      Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, DoubleSupplier precision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;

    m_x = x;
    m_y = y;
    m_theta = theta;
    m_precision = precision;

    turnController.enableContinuousInput(-180, 180);

    addRequirements(drivetrain);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRed = DriverStation.getAlliance().get() == Alliance.Red;
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_precisionFactor = Math.pow(kDrivingPrecisionMultiplier, m_precision.getAsDouble());
    double Y = MathUtil.applyDeadband(m_y.getAsDouble(), kDriveDeadband) * m_precisionFactor;
    double X = MathUtil.applyDeadband( m_x.getAsDouble(), kDriveDeadband) * m_precisionFactor;
    double rot = MathUtil.applyDeadband(m_theta.getAsDouble(), kDriveDeadband) * m_precisionFactor;
    
    var speedAdjustmentFactor = kMaxSpeedMetersPerSecond * kSpeedMultiplier;
    m_xSpeed = -m_xLimiter.calculate(Y * Y * Math.signum(Y) * speedAdjustmentFactor);
    
    m_ySpeed = -m_yLimiter.calculate(X * X * Math.signum(X) * speedAdjustmentFactor);
    
    m_thetaSpeed = -m_thetaLimiter.calculate(rot * rot * Math.signum(rot) * kMaxAngularSpeedRadiansPerSecond * kSpeedMultiplier * kRotationSpeedMultiplier);
    
    if (isRed) {
      m_drivetrain.drive(-m_xSpeed, -m_ySpeed, m_thetaSpeed, true);
    } else {
      m_drivetrain.drive(m_xSpeed, m_ySpeed, m_thetaSpeed, true);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
