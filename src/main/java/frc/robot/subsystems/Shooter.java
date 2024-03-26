// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANConstants.*;
import static frc.robot.Constants.Shooter.*;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_shooterMotorRight = new CANSparkMax(kRightShooterMotorID, MotorType.kBrushless); 
  private final CANSparkMax m_shooterMotorLeft = new CANSparkMax(kLeftShooterMotorID, MotorType.kBrushless);
  private boolean isRed;
  

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotorRight.setInverted(true); //false for big shooter
    m_shooterMotorLeft.setInverted(false);
    m_shooterMotorLeft.getEncoder().setVelocityConversionFactor(1.);
    m_shooterMotorRight.getEncoder().setVelocityConversionFactor(1.);
    m_shooterMotorLeft.burnFlash();
    m_shooterMotorRight.burnFlash();
    SmartDashboard.putNumber("Shot Speed", 950.);
    if (DriverStation.getAlliance().get()==Alliance.Red) {
      isRed = true;
    } else {
      isRed = false;
    }
    Shuffleboard.getTab("Matches").addNumber("Left Shooter RPM", () -> getLeftShooterSpeed())
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(4, 1)
      .withSize(2, 2)
      .withProperties(Map.of("min", -3000, "max", 5676, "orientation", "VERTICAL"))
      ;

    Shuffleboard.getTab("Matches").addNumber("Right Shooter RPM", () -> getRightShooterSpeed())
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(6, 1)
      .withSize(2, 2)
      .withProperties(Map.of("min", -3000, "max", 5676, "orientation", "VERTICAL"))
      ;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Shooter Speed", getLeftShooterSpeed());
    SmartDashboard.putNumber("Right Shooter Speed", getRightShooterSpeed());
    SmartDashboard.putNumber("Left Shooter Current", m_shooterMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("Right Shooter Current", m_shooterMotorRight.getOutputCurrent());
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(Double speed) {
    if (isRed) {
      m_shooterMotorRight.set(0.75 * speed/kMaxSpeedRPM);
      m_shooterMotorLeft.set(speed/kMaxSpeedRPM);
    } else {
      m_shooterMotorRight.set(speed/kMaxSpeedRPM);
      m_shooterMotorLeft.set(0.75 * speed/kMaxSpeedRPM);
    }
    
  }

  public void stopShooter() {
    m_shooterMotorRight.set(0);
    m_shooterMotorLeft.set(0);

  }

  public void setRightShooterSpeed(Double speed) {
    m_shooterMotorRight.set(speed/kMaxSpeedRPM);
  }
  
  public void setLeftShooterSpeed(Double speed) {
    m_shooterMotorLeft.set(speed/kMaxSpeedRPM);

  }

  public double getRightShooterSpeed() {
    return m_shooterMotorRight.getEncoder().getVelocity();  
  }
  
  public double getLeftShooterSpeed() {
    return m_shooterMotorLeft.getEncoder().getVelocity();  
  }
}
