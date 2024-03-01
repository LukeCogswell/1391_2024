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
import static frc.robot.Constants.Swerve.PID.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Shooter.*;


public class TrackWhileMoving extends Command {
  private Drivetrain m_drivetrain;
  private PIDController turnController = new PIDController(0.02, kTurnI, 0.0);
  boolean m_PIDcontrol, isRed;
  DoubleSupplier m_x, m_y, m_precision;
  double m_toAngle, m_xSpeed, m_ySpeed, m_thetaSpeed;

  boolean m_holdAngle = false;


  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(kDriveSlewRateLimit);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(kDriveSlewRateLimit);
  private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(kthetaSlewRateLimit);

  /** Creates a new TrackWhileMoving. */
  public TrackWhileMoving(Drivetrain drivetrain, DoubleSupplier x_speed, DoubleSupplier y_speed, DoubleSupplier precision) {
    m_drivetrain = drivetrain;
    m_precision = precision;
    m_x = x_speed;
    m_y = y_speed;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRed = DriverStation.getAlliance().get() == Alliance.Red;
    turnController.enableContinuousInput(-180, 180);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("xSpeed", X);
    double m_precisionFactor = Math.pow(kDrivingPrecisionMultiplier, m_precision.getAsDouble());
    double Y = MathUtil.applyDeadband(m_y.getAsDouble(), kDriveDeadband) * m_precisionFactor;
    double X = MathUtil.applyDeadband( m_x.getAsDouble(), kDriveDeadband) * m_precisionFactor;
    
    var speedAdjustmentFactor = kMaxSpeedMetersPerSecond * kSpeedMultiplier;
    m_xSpeed = -m_xLimiter.calculate(Y * Y * Math.signum(Y) * speedAdjustmentFactor);
    
    m_ySpeed = -m_yLimiter.calculate(X * X * Math.signum(X) * speedAdjustmentFactor);
    // var dis = m_drivetrain.getDistanceToSpeaker();
    // var dDis = m_drivetrain.getChangeInDistanceToSpeaker(m_xSpeed, m_ySpeed);
    
    var dTheta = -m_drivetrain.getChangeInAngleToSpeaker(m_xSpeed, m_ySpeed);
    dTheta = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? -dTheta : dTheta;
  
    // SmartDashboard.putNumber("Angle", m_drivetrain.getAngleToSpeaker());
    // SmartDashboard.putNumber("dTheta", dTheta);
    
    var rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees() - (m_drivetrain.getAngleToSpeaker() + kShootingRotationAdjustmentMultiplier * dTheta));
    
    if (isRed) {
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
