// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Loader extends SubsystemBase {
  /** Creates a new Loader. */
  private final CANSparkMax m_loaderMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final AnalogInput m_beamBreakSensor = new AnalogInput(0);
  public Loader() {
    m_loaderMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Note In Shooter", hasNoteInShooter());
  }

  public void setLoaderMotor(Double power) {
    m_loaderMotor.set(power);
  }

  public boolean hasNoteInShooter() {
    return m_beamBreakSensor.getValue() <= 30;
  }

}
