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
  private final CANSparkMax m_shooterMotorTop = new CANSparkMax(13, MotorType.kBrushless); 
  private final CANSparkMax m_shooterMotorBottom = new CANSparkMax(14, MotorType.kBrushless);
  

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotorTop.setInverted(true);
    m_shooterMotorBottom.setInverted(true);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("TopShooterSpeed", getTopShooterSpeed());
    // SmartDashboard.putNumber("BotShooterSpeed", getBottomShooterSpeed());
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(Double speed) {
    m_shooterMotorTop.set(speed/kMaxSpeedRPM);
    m_shooterMotorBottom.set(speed/kMaxSpeedRPM);
    
  }

  public void stopShooter() {
    m_shooterMotorTop.set(0);
    m_shooterMotorBottom.set(0);

  }

  public void setTopShooterSpeed(Double speed) {
    m_shooterMotorTop.set(speed/kMaxSpeedRPM);
  }
  
  public void setBottomShooterSpeed(Double speed) {
    m_shooterMotorBottom.set(speed/kMaxSpeedRPM);

  }

  public double getTopShooterSpeed() {
    SmartDashboard.putNumber("Top RPM", m_shooterMotorTop.getEncoder().getVelocity() );
    return m_shooterMotorTop.getEncoder().getVelocity();  
  }
  
  public double getBottomShooterSpeed() {
    SmartDashboard.putNumber("Bottom RPM", m_shooterMotorBottom.getEncoder().getVelocity() );
    return m_shooterMotorBottom.getEncoder().getVelocity();  
  }
}
