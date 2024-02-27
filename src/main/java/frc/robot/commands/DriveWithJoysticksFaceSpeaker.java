// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Swerve.PID.kTurnD;
import static frc.robot.Constants.Swerve.PID.kTurnI;
import static frc.robot.Constants.Swerve.PID.kTurnP;
import static frc.robot.Constants.MeasurementConstants.*;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveWithJoysticksFaceSpeaker extends Command {

  Drivetrain m_drivetrain;

  DoubleSupplier m_x, m_y, m_precision;

  boolean m_PIDcontrol, isRed;

  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);

  double m_xSpeed, m_ySpeed, m_thetaSpeed, m_precisionFactor, Y, X, rot;

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(kDriveSlewRateLimit);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(kDriveSlewRateLimit);
  private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(kthetaSlewRateLimit);
  /** Creates a new Drive. */
  public DriveWithJoysticksFaceSpeaker(
      Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier precision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;

    m_x = x;
    m_y = y;
    m_precision = precision;

    addRequirements(drivetrain);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.setSetpoint(0.);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    X = m_x.getAsDouble();
    X = MathUtil.applyDeadband(X, kDriveDeadband);
    Y = m_y.getAsDouble();
    Y = MathUtil.applyDeadband(Y, kDriveDeadband);


   var m_precisionFactor = Math.pow(kDrivingPrecisionMultiplier, m_precision.getAsDouble());
    SmartDashboard.putNumber("PRecision Factor", m_precisionFactor);
    
    var speedAdjustmentFactor = kMaxSpeedMetersPerSecond * kSpeedMultiplier;
    m_xSpeed = -m_xLimiter.calculate(Y * Y * Math.signum(Y) * speedAdjustmentFactor);
    
    m_ySpeed = -m_yLimiter.calculate(X * X * Math.signum(X) * speedAdjustmentFactor);

    m_thetaSpeed = -turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees() - m_drivetrain.getAngleToSpeaker());
    m_thetaSpeed = MathUtil.clamp(m_thetaSpeed, -kMaxAngularSpeedRadiansPerSecond * kSpeedMultiplier, kMaxAngularSpeedRadiansPerSecond * kSpeedMultiplier);
    m_thetaSpeed = m_thetaLimiter.calculate(m_thetaSpeed);

    m_drivetrain.drive(m_xSpeed, m_ySpeed, m_thetaSpeed, false);

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
