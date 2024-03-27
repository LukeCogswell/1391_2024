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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

import static frc.robot.Constants.Shooter.PID.*;
import static frc.robot.Constants.LEDs.kConfidentShotRange;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Swerve.PID.*;

public class ShootWhileMoving extends Command {
  private Drivetrain m_drivetrain;
  private Shooter m_shooter;
  private Turret m_turret;
  private Loader m_loader;
  private LEDs m_leds;
  private PIDController angleController = new PIDController(kAngleP, kAngleI, kAngleD);
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
  private PIDController LLturnController = new PIDController(kLLTurnP, kLLTurnI, kLLTurnD);
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private Boolean isRed;
  private DoubleSupplier xSpeed, ySpeed, m_precision;
  private Double X, Y, rot, m_precisionFactor, m_xSpeed, m_ySpeed, dis;
  /** Creates a new ShootWhileMoving. */
  public 
  ShootWhileMoving(Drivetrain drivetrain, Shooter shooter, Turret turret, Loader loader, DoubleSupplier x_speed, DoubleSupplier y_speed, DoubleSupplier precision, LEDs leds) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_loader = loader;
    m_turret = turret;
    m_precision = precision;
    xSpeed = x_speed;
    ySpeed = y_speed;
    m_leds = leds;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, drivetrain, loader, turret, leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      isRed = true;
    } else {
      isRed = false;
    }
    m_precisionFactor = Math.pow(0.2, m_precision.getAsDouble());
    X = xSpeed.getAsDouble();
    Y = ySpeed.getAsDouble();
    X = X > kAutoDriveSpeedLimiter ? kAutoDriveSpeedLimiter : X;
    X = X < -kAutoDriveSpeedLimiter ? -kAutoDriveSpeedLimiter : X;
    Y = Y > kAutoDriveSpeedLimiter ? kAutoDriveSpeedLimiter : Y;
    Y = Y < -kAutoDriveSpeedLimiter ? -kAutoDriveSpeedLimiter : Y;
    turnController.setTolerance(3);
    turnController.setSetpoint(0.0);
    turnController.enableContinuousInput(-180, 180);
    LLturnController.setSetpoint(0.);
    LLturnController.setTolerance(3);
    angleController.setSetpoint(0);
    angleController.setTolerance(0.7);
    if (X+Y >= 0.6) {
      turnController.setTolerance(5);
      LLturnController.setTolerance(5 );
      angleController.setTolerance(1.5);
    }
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

    var dDis = m_drivetrain.getChangeInDistanceToSpeaker(m_xSpeed, m_ySpeed);
    
    // var dTheta = -m_drivetrain.getChangeInAngleToSpeaker(m_xSpeed, m_ySpeed);
    // dTheta = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? -dTheta : dTheta;
    // dTheta = MathUtil.clamp(dTheta, -15, 15);
    
    // SmartDashboard.putNumber("Angle", m_drivetrain.getAngleToSpeaker());
    // SmartDashboard.putNumber("dTheta", dTheta);

    dis = m_drivetrain.getDistanceToSpeaker();

    if (m_drivetrain.getTV()) {
      dis = m_drivetrain.getDistanceToSpeakerAprilTag(); 
      // double Ryaw = m_drivetrain.getFieldPosition().getRotation().getDegrees(); 
      // if (isRed) {
      //   if (Ryaw >= -160 && Ryaw < 0.) {
      //     LLturnController.setSetpoint(2);
      //   } else if (Ryaw <= 160 && Ryaw > 0.) {
      //     LLturnController.setSetpoint(-2);
      //   } else {
      //     LLturnController.setSetpoint(0.);
      //   }
      // } else {
      //   if (Ryaw <= -20) {
      //     LLturnController.setSetpoint(2);
      //   } else if (Ryaw >= 20) {
      //     LLturnController.setSetpoint(-2);
      //   } else {
      //     LLturnController.setSetpoint(0.);
      //   }

      // }
      rot = LLturnController.calculate(m_drivetrain.getTX());
    } else {
      // rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees() - (m_drivetrain.getAngleToSpeaker() /*+ kShootingRotationAdjustmentMultiplier * dTheta*/));
      rot = 0.;
      dis = m_drivetrain.getDistanceToSpeaker();
    }

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
    
    // var spinMultiplier = 0.75;
    // var spinMultiplier = 0.85 / distanceMultiplier;
    // spinMultiplier = spinMultiplier > 1 ? 1 : spinMultiplier;
    // spinMultiplier = 1.;

    m_shooter.setShooterSpeed(distanceMultiplier * 5676.);
    // m_shooter.setRightShooterSpeed(distanceMultiplier * 5676.0);
    // m_shooter.setLeftShooterSpeed(distanceMultiplier * 5676.0 * spinMultiplier);

    // SmartDashboard.putNumber("AnglePID", angleController.calculate((m_shooter.getRequiredShooterAngle(dis, dDis)*180 / Math.PI) - m_shooter.getShooterAngle()));
    // SmartDashboard.putNumber("required angle", m_shooter.getRequiredShooterAngle(dis, dDis)*180 / Math.PI);


    m_turret.setAngleMotor(-(angleController.calculate((m_turret.getRequiredShooterAngleFromTable(dis)) - m_turret.getShooterAngle())));

    SmartDashboard.putNumber("Required Angle", m_turret.getRequiredShooterAngleFromTable(dis) * 180 / Math.PI);

    
    if (angleController.atSetpoint()) {
      m_leds.setTopThird(Color.kGreen);
    } else {
      m_leds.setTopThird(Color.kGray);
    }
    if (m_shooter.getRightShooterSpeed() >= distanceMultiplier * 5676.0 * 0.8) {
      m_leds.setMiddleThird(Color.kGreen);
    } else {
      m_leds.setMiddleThird(Color.kAliceBlue);
    }
    if (LLturnController.atSetpoint()) {
      m_leds.setBottomThird(Color.kGreen);
    } else {
      m_leds.setBottomThird(Color.kRed);
    }

    m_leds.start();

    SmartDashboard.putBoolean("Angle?", angleController.atSetpoint());
    SmartDashboard.putBoolean("LLTurn?", LLturnController.atSetpoint());
    SmartDashboard.putBoolean("Turn?", turnController.atSetpoint());
    SmartDashboard.putBoolean("Speed?", m_shooter.getRightShooterSpeed() >= distanceMultiplier * 5676.0 * 0.8);
    
    m_drivetrain.drive(m_xSpeed, m_ySpeed, rot, true);
    if (angleController.atSetpoint() && m_shooter.getRightShooterSpeed() >= distanceMultiplier * 5676.0 * 0.8 && LLturnController.atSetpoint()) {
      m_loader.setLoaderMotor(1.);
    } else {
      m_loader.stop();
    }
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
