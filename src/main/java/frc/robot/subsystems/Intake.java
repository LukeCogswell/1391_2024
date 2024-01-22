// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_motorRight = new CANSparkMax(9, MotorType.kBrushless); 
  private final CANSparkMax m_motorLeft = new CANSparkMax(10, MotorType.kBrushless); 
  private final Rev2mDistanceSensor distSensor = new Rev2mDistanceSensor(com.revrobotics.Rev2mDistanceSensor.Port.kOnboard);
  private NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-twelve");
  /** Creates a new Intake. */
  public Intake() {
    m_motorLeft.setInverted(true);
    m_motorRight.setInverted(true);
    distSensor.setAutomaticMode(true);
    distSensor.setDistanceUnits(Unit.kMillimeters);
  }

  @Override
  public void periodic() {
    // if(distSensor.isRangeValid()) {
    //   SmartDashboard.putNumber("Range Onboard", distSensor.getRange());
    //   SmartDashboard.putNumber("Timestamp Onboard", distSensor.getTimestamp());
    // }
    SmartDashboard.putBoolean("HasNote?", hasNoteInIntake());
    SmartDashboard.putBoolean("CurrentHasNote?", currentHasNoteInIntake());
    SmartDashboard.putNumber("Bot Int Current", getBottomIntakeCurrentDraw());
    SmartDashboard.putNumber("Top Int Current", getTopIntakeCurrentDraw());
    // This method will be called once per scheduler run
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
    m_motorRight.set(power);
    m_motorLeft.set(power);
  }

  public boolean currentHasNoteInIntake() {
    return getBottomIntakeCurrentDraw() >= 15.;
  }

  public double getBottomIntakeCurrentDraw() {
    return m_motorLeft.getOutputCurrent();
  }

  public double getTopIntakeCurrentDraw() {
    return m_motorRight.getOutputCurrent();
  }

  public boolean hasNoteInIntake() {
    if (distSensor.isRangeValid()) {
      return distSensor.getRange() <= 400.;
    }
    return false; 
  }

}
