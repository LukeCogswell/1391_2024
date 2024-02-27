// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.Constants.Elevator.*;
import static frc.robot.Constants.CANConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final CANSparkMax m_elevatorMotorLeft = new CANSparkMax(kLeftElevatorMotorID, MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotorRight = new CANSparkMax(kRightElevatorMotorID, MotorType.kBrushless);
  /** Creates a new Elevator. */
  public Elevator() {
    m_elevatorMotorLeft.setInverted(true);
    m_elevatorMotorRight.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LELEVATOR CURRETN", m_elevatorMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("RELEVATOR CURRETN", m_elevatorMotorRight.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Height L", getElevatorHeightL());
    SmartDashboard.putNumber("Elevator Height R", getElevatorHeightR());
    // This method will be called once per scheduler run
  }

  public double getElevatorHeightR() {
    return m_elevatorMotorRight.getEncoder().getPosition() * kMotorRotationsToMeters;
  }
  public double getElevatorHeightL() {
    return m_elevatorMotorLeft.getEncoder().getPosition() * kMotorRotationsToMeters;
  }

  public void setElevator(Double power) {
    m_elevatorMotorLeft.set(power);
    m_elevatorMotorRight.set(power);
  }

  public void stop() {
    m_elevatorMotorLeft.set(0.);
    m_elevatorMotorRight.set(0.);
  }

}
