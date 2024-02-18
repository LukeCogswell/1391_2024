// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final CANSparkMax m_climberMotor = new CANSparkMax(16, MotorType.kBrushless);

  /** Creates a new Climber. */
  public Climber() {
    m_climberMotor.setInverted(false);
  }
  public void runClimber(Double speed) {
    m_climberMotor.set(MathUtil.clamp(speed, 0, 1));
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Current", m_climberMotor.getOutputCurrent());
    // This method will be called once per scheduler run
  }
}
