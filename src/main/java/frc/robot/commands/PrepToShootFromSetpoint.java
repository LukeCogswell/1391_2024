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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Swerve.PID.*;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.MeasurementConstants.kMaxAngularSpeedRadiansPerSecond;
import static frc.robot.Constants.MeasurementConstants.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.Shooter.PID.*;

import java.util.function.DoubleSupplier;

public class PrepToShootFromSetpoint extends Command {
  /** Creates a new ShootNote. */
  private Drivetrain m_drivetrain;
  private Shooter m_shooter;
  private Turret m_turret;
  private Loader m_loader;

  private Double shotSpeed, angle, m_xSpeed, m_ySpeed, rotSetpoint, rot;

  private Trigger shotTrigger;
  private DoubleSupplier xIn, yIn, precisionIn;
  private Boolean isRed;

  private PIDController angleController = new PIDController(kAngleP, kAngleI, kAngleD);
  private PIDController turnController = new PIDController(.02, kTurnI, 0);

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(kDriveSlewRateLimit);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(kDriveSlewRateLimit);
  private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(kthetaSlewRateLimit);

  public PrepToShootFromSetpoint(Double speed, Double Angle, Double rotation, Drivetrain drivetrain, Shooter shooter, Turret turret, Loader loader, Trigger shootingTrigger, DoubleSupplier x_input, DoubleSupplier y_input, DoubleSupplier precision_input) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_turret = turret;
    m_loader = loader;
    shotSpeed = speed;
    angle = Angle;
    shotTrigger = shootingTrigger;
    xIn = x_input;
    yIn = y_input;
    precisionIn = precision_input;
    rotSetpoint = rotation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, loader, turret, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRed = DriverStation.getAlliance().get() == Alliance.Red;
    angleController.setSetpoint(angle);
    angleController.setTolerance(.5);
    turnController.setSetpoint(rotSetpoint);
    turnController.enableContinuousInput(-180, 180);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double m_precisionFactor = Math.pow(kDrivingPrecisionMultiplier, precisionIn.getAsDouble());
    double Y = MathUtil.applyDeadband(yIn.getAsDouble(), kDriveDeadband) * m_precisionFactor;
    double X = MathUtil.applyDeadband( xIn.getAsDouble(), kDriveDeadband) * m_precisionFactor;
    
    var speedAdjustmentFactor = kMaxSpeedMetersPerSecond * kSpeedMultiplier;
    m_xSpeed = -m_xLimiter.calculate(Y * Y * Math.signum(Y) * speedAdjustmentFactor);
    m_ySpeed = -m_yLimiter.calculate(X * X * Math.signum(X) * speedAdjustmentFactor);
    
    rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees());

    if (isRed) {
      m_drivetrain.drive(-m_xSpeed, -m_ySpeed, rot, true);
    } else {
      m_drivetrain.drive(m_xSpeed, m_ySpeed, rot, true);
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
