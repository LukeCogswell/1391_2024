// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import static frc.robot.Constants.Elevator.*;

import java.util.Map;

import static frc.robot.Constants.CANConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final CANSparkMax m_elevatorMotorLeft = new CANSparkMax(kLeftElevatorMotorID, MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotorRight = new CANSparkMax(kRightElevatorMotorID, MotorType.kBrushless);
  private Double heightOffset = 0.;
  /** Creates a new Elevator. */
  public Elevator() {
    m_elevatorMotorLeft.setInverted(false);
    m_elevatorMotorRight.setInverted(true);
    Shuffleboard.getTab("Matches").addNumber("Left Elevator", () -> getElevatorHeightL())
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(6, 1)
      .withSize(1, 2)
      .withProperties(Map.of("min", 0, "max", 8.7, "orientation", "VERTICAL"))
      ;

    Shuffleboard.getTab("Matches").addNumber("Right Elevator", () -> getElevatorHeightR())
      .withWidget(BuiltInWidgets.kNumberBar)
      .withPosition(7, 1)
      .withSize(1, 2)
      .withProperties(Map.of("min", 0, "max", 8.7, "orientation", "VERTICAL"))
      ;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("L ELEVATOR CURRENT L", m_elevatorMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber("R ELEVATOR CURRENT R", m_elevatorMotorRight.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Height L", getElevatorHeightL());
    SmartDashboard.putNumber("Elevator Height R", getElevatorHeightR());
    SmartDashboard.putBoolean("ELEVATORDOWN?", m_elevatorMotorRight.getReverseLimitSwitch(Type.kNormallyOpen).isPressed());
    // if (m_elevatorMotorRight.getReverseLimitSwitch(Type.kNormallyOpen).isPressed()) {
    //   heightOffset = getElevatorHeightR() + heightOffset;
    // }b
    // This method will be called once per scheduler run
  }

  public double getElevatorHeightR() {
    return m_elevatorMotorRight.getEncoder().getPosition() * kMotorRotationsToMeters - heightOffset;
  }
  public double getElevatorHeightL() {
    return m_elevatorMotorLeft.getEncoder().getPosition() * kMotorRotationsToMeters - heightOffset;
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
