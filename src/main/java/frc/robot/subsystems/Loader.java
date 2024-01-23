// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Loader extends SubsystemBase {
  /** Creates a new Loader. */
  private final CANSparkMax m_loaderMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kMXP);
  public Loader() {
    m_loaderMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IR", getColorProximity());
    SmartDashboard.putBoolean("Note In Shooter", hasNoteInShooter());
  }

  public void setLoaderMotor(Double power) {
    m_loaderMotor.set(power);
  }

  public double getColorProximity() {
    return m_colorSensor.getProximity();
  }

  public boolean hasNoteInShooter() {
    return getColorProximity() >= 300;
  }

}
