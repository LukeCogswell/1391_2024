// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_motorRight = new CANSparkMax(9, MotorType.kBrushless); 
  private final CANSparkMax m_motorLeft = new CANSparkMax(10, MotorType.kBrushless); 
  /** Creates a new Intake. */
  public Intake() {
    m_motorLeft.setInverted(true);
    m_motorRight.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntake(Double power) {
    m_motorRight.set(power);
    m_motorLeft.set(power);
  }

}
