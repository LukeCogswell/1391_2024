// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANConstants.*;

import java.util.Map;

public class Intake extends SubsystemBase {
  private final CANSparkMax m_intakeMotor = new CANSparkMax(kIntakeBeltMotorID, MotorType.kBrushless); 
  private NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-twelve");
  private AnalogInput m_beamBreakSensor = new AnalogInput(2);

  /** Creates a new Intake. */
  public Intake() {
    m_intakeMotor.setInverted(false);
    m_intakeMotor.setOpenLoopRampRate(0.);
    Shuffleboard.getTab("Matches").addBoolean("Note TV", () -> getTV())
      .withPosition(0,1)
      .withSize(2, 2)
      .withProperties(Map.of("colorWhenFalse", "black", "colorWhenTrue", "orange"))
    ;
    Shuffleboard.getTab("Testing").addBoolean("Intake Current Boolean", () -> currentHasNoteInIntake())
      .withPosition(3,5)
      .withSize(2, 1)
      .withProperties(Map.of("colorWhenFalse", "black", "colorWhenTrue", "orange"))
    ;
    Shuffleboard.getTab("Testing").addNumber("Intake Current", () -> getIntakeCurrentDraw())
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(0,3)
      .withSize(3, 3)
    ;
    Shuffleboard.getTab("Testing").addNumber("Intake Speed", () -> m_intakeMotor.getEncoder().getVelocity())
      .withWidget(BuiltInWidgets.kGraph)
      .withPosition(3,3)
      .withSize(2, 2)
    ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("TY", getTY());
    SmartDashboard.putBoolean("HasNote?", hasNoteInIntake());
    SmartDashboard.putBoolean("NOTE TV", getTV());
    // SmartDashboard.putNumber("bELTS cURRNET", m_intakeMotor.getOutputCurrent());
    // SmartDashboard.putBoolean("CurrentHasNote?", currentHasNoteInIntake());
    
    // SmartDashboard.putNumber("Bot Int Current", getIntakeCurrentDraw());
    // SmartDashboard.putNumber("BBSEnsorIntake", m_beamBreakSensor.getValue());
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
    return m_intakeMotor.getOutputCurrent() >= 17.;
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
