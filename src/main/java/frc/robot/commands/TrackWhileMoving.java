// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.SwerveModuleConstants.*;

public class TrackWhileMoving extends Command {
  private Drivetrain m_drivetrain;
  private Shooter m_shooter;
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(50);
  private Double xSpeed, ySpeed, rot, m_precisionFactor;

  /** Creates a new TrackWhileMoving. */
  public TrackWhileMoving(Drivetrain drivetrain, Shooter shooter, Double x_speed, Double y_speed, Double precision) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    xSpeed = x_speed;
    ySpeed = y_speed;
    m_precisionFactor = precision;
    addRequirements(drivetrain, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.setSetpoint(0.0);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed =
      -m_xLimiter.calculate(MathUtil.applyDeadband(Math.pow(ySpeed, 2) * Math.signum(ySpeed), kDriveDeadband))
      * kMaxSpeedMetersPerSecond * kSpeedMultiplier * m_precisionFactor;
    
    ySpeed =
      -m_yLimiter.calculate(MathUtil.applyDeadband(Math.pow(xSpeed, 2) * Math.signum(xSpeed), kDriveDeadband))
      * kMaxSpeedMetersPerSecond * kSpeedMultiplier * m_precisionFactor;
    var dis = m_drivetrain.getDistanceToSpeaker();
    var dDis = m_drivetrain.getChangeInDistanceToSpeaker(xSpeed, ySpeed);

    var dTheta = m_drivetrain.getChangeInAngleToSpeaker(xSpeed, ySpeed);
    rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees() - m_drivetrain.getAngleToSpeaker() + dTheta);

    SmartDashboard.putNumber("ShooterAngle", m_shooter.getRequiredShooterAngle(dis));
    SmartDashboard.putNumber("ShooterChangeInAngle", m_shooter.getChangeInShooterAngle(dis, dDis));

    m_drivetrain.drive(xSpeed, ySpeed, rot);

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
