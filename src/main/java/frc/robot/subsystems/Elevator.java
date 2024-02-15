// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.Elevator.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(15, MotorType.kBrushless);
  /** Creates a new Elevator. */
  public Elevator() {
    m_elevatorMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
    // This method will be called once per scheduler run
  }

  public double getElevatorHeight() {
    return m_elevatorMotor.getEncoder().getPosition() * kMotorRotationsToMeters;
  }

  public void setElevator(Double power) {
    m_elevatorMotor.set(power);
  }

  public void stop() {
    m_elevatorMotor.set(0.);
  }

}
