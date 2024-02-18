// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeMotor = new CANSparkMax(10, MotorType.kBrushless); 
  private NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-twelve");
  private AnalogInput m_beamBreakSensor = new AnalogInput(1);

  /** Creates a new Intake. */
  public Intake() {
    m_intakeMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("HasNote?", hasNoteInIntake());
    SmartDashboard.putBoolean("CurrentHasNote?", currentHasNoteInIntake());
    // SmartDashboard.putNumber("Bot Int Current", getIntakeCurrentDraw());
    SmartDashboard.putNumber("BBSEnsorIntake", m_beamBreakSensor.getValue());
    // SmartDashboard.putNumber("Top Int Current", getTopIntakeCurrentDraw());
  }

  public boolean getTV() {
    return m_limelight.getEntry("tv").getDouble(0.0) == 1.;
  }

  public double getTX() {
    return m_limelight.getEntry("tx").getDouble(0.0);
  }

  public double getTY() {
    return m_limelight.getEntry("ty").getDouble(0.0);
  }

  public void setIntake(Double power) {
    m_intakeMotor.set(power);
  }

  public boolean currentHasNoteInIntake() {
    return getIntakeCurrentDraw() >= 11.;
  }

  public double getIntakeCurrentDraw() {
    return m_intakeMotor.getOutputCurrent();
  }

  public void stop() {
    m_intakeMotor.set(0.);
  }

  public boolean hasNoteInIntake() {
    return m_beamBreakSensor.getValue() <= 30;
  }

}
