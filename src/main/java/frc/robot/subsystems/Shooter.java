// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Shooter.*;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_shooterMotorRight = new CANSparkMax(14, MotorType.kBrushless); 
  private final CANSparkMax m_shooterMotorLeft = new CANSparkMax(13, MotorType.kBrushless);
  

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotorRight.setInverted(false); //false for big shooter
    m_shooterMotorLeft.setInverted(true);
    m_shooterMotorLeft.getEncoder().setVelocityConversionFactor(1.);
    m_shooterMotorRight.getEncoder().setVelocityConversionFactor(1.);
    m_shooterMotorLeft.burnFlash();
    m_shooterMotorRight.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Shooter Speed", getLeftShooterSpeed());
    SmartDashboard.putNumber("Right Shooter Speed", getRightShooterSpeed());
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(Double speed) {
    m_shooterMotorRight.set(speed/kMaxSpeedRPM);
    m_shooterMotorLeft.set(speed/kMaxSpeedRPM);
    
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
