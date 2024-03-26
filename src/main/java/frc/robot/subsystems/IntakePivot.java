// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch.Type;

import static frc.robot.Constants.CANConstants.kIntakePivotMotorID;
import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Shooter.kAngleEncoderOffset;

import java.util.Map;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
  private final CANSparkMax m_pivotMotor = new CANSparkMax(kIntakePivotMotorID, MotorType.kBrushless);
  private final DutyCycleEncoder m_intakeEncoder = new DutyCycleEncoder(1); 
  /** Creates a new IntakePivot. */
  public IntakePivot() {
    m_pivotMotor.setInverted(true);
    m_pivotMotor.setIdleMode(IdleMode.kBrake);
    Shuffleboard.getTab("Matches").addNumber("Intake Angle", () -> getIntakeAngle()).withWidget(BuiltInWidgets.kGyro)
      .withPosition(0, 3)
      .withSize(4, 4)
      .withProperties(Map.of("startingAngle", kMaxRotation+180, "counterClockwise", true))
      ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("KMAXDOWN",  Math.abs((270 - getIntakeAngle()) / 100.));
    SmartDashboard.putNumber("INTAKE CURRENT", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake Angle", getIntakeAngle());
    SmartDashboard.putBoolean("IsUp?", isUp());
    SmartDashboard.putBoolean("IsDown?", isDown());
  }

  public void stop() {
    m_pivotMotor.set(0.);
  }

  public void setAngleMotor(Double power) {
    m_pivotMotor.set(power);
  }

  public double getIntakeAngle() {
    return (((m_intakeEncoder.get()) * kEncoderGearRatio * 360) - kIntakeEncoderOffset);
  }

  public boolean isDown() {
    return m_pivotMotor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
  }

  public boolean isUp() {
    return m_pivotMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
  }

}
