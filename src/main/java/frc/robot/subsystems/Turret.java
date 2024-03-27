// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Shooter.PID.*;
import static frc.robot.Constants.Shooter.RangeTableAprilTag.*;

import java.util.Map;

import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.CANConstants.*;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final CANSparkMax m_angleMotor = new CANSparkMax(kTurretMotorID, MotorType.kBrushless);
  private DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(0);
  private InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
  public SimpleWidget p, i, d;
  public Boolean isAngled = false;

  public Turret() {
    // SmartDashboard.putNumber("P", kAngleP);
    // SmartDashboard.putNumber("I", kAngleI);
    // SmartDashboard.putNumber("D", kAngleD);
    configureMotorController();
    angleMap.put(entry0[0], entry0[1]);
    angleMap.put(entry1[0], entry1[1]);
    angleMap.put(entry2[0], entry2[1]);
    angleMap.put(entry3[0], entry3[1]);
    angleMap.put(entry4[0], entry4[1]);
    angleMap.put(entry40[0], entry40[1]);
    angleMap.put(entry5[0], entry5[1]);
    angleMap.put(entry6[0], entry6[1]);
    angleMap.put(entry7[0], entry7[1]);
    angleMap.put(entry8[0], entry8[1]);
    angleMap.put(entry9[0], entry9[1]);
    angleMap.put(entry10[0], entry10[1]);
    angleMap.put(entry11[0], entry11[1]);
    Shuffleboard.getTab("Testing").addNumber("Turret Current", () -> getCurrent()).withWidget(BuiltInWidgets.kGraph);
    Shuffleboard.getTab("Testing").addNumber("Turret Speed", () -> getVelocity()).withWidget(BuiltInWidgets.kGraph);
    Shuffleboard.getTab("Matches").addNumber("Turret Angle", () -> getShooterAngle()).withWidget(BuiltInWidgets.kGyro)
      .withPosition(4, 3)
      .withSize(4, 3)
      .withProperties(Map.of("startingAngle", 90, "counterClockwise", true))
      ;
    p = Shuffleboard.getTab("Testing").add("Turret P", kAngleP)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withSize(7, 1)
      .withPosition(6, 2)
      .withProperties(Map.of("min", 0, "max", 0.1))
      ;
      i = Shuffleboard.getTab("Testing").add("Turret I", kAngleI)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withSize(7, 1)
      .withPosition(6, 3)
      .withProperties(Map.of("min", 0, "max", 0.1))
      ;
      d = Shuffleboard.getTab("Testing").add("Turret D", kAngleD)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withSize(7, 1)
      .withPosition(6, 4)
      .withProperties(Map.of("min", 0, "max", 0.1))
      ;
  }

  @Override
  public void periodic() {
    // System.out.println(m_angleEncoder.getAbsolutePosition());
    // System.out.println("FALSE");
    // System.out.println(getShooterAngle());
    SmartDashboard.putNumber("Angle", getShooterAngle());
    // This method will be called once per scheduler run
  }

  public double getVelocity() {
    return m_angleMotor.getEncoder().getVelocity();
  }

  public double getCurrent() {
    return m_angleMotor.getOutputCurrent();
  }

  public void setMotor(Double pwr) {
    m_angleMotor.set(pwr);
  }

  private void configureMotorController() {
    m_angleMotor.setInverted(true);
    m_angleMotor.setIdleMode(IdleMode.kBrake);
    m_angleMotor.setSmartCurrentLimit(30);
    // m_angleMotor.setIdleMode(IdleMode.kCoast);
    // m_angleMotor.setSmartCurrentLimit(60);
    m_angleMotor.burnFlash();
  }

  public void setAngleMotor(Double power) {
    var pwr = power;
    // var angle = getShooterAngle();
    // if (angle >= -181 && angle <= -60) {
    //   pwr = MathUtil.clamp(pwr, -1., 0.);
    // } else if (angle <= -32. && angle >= -50) {
    //   pwr = MathUtil.clamp(pwr, 0., 1.);
    // }
    m_angleMotor.set(pwr);
    // if (getShooterAngle() <= -15 || getShooterAngle() >= 80) {
    //   m_angleMotor.set(0.);

    // } else {
    //   m_angleMotor.set(power);
    // }
  }

  public void stop() {
    m_angleMotor.set(0.);
  }

  public double getRequiredShooterAngle(Double distanceToSpeaker, Double dDistanceToSpeaker) {
    var theta1 = Math.atan((((kSpeakerOpeningMaxHeight + kSpeakerOpeningMinHeight)/2) - shooterReleaseHeight) / (distanceToSpeaker-0.23));
    var theta = theta1 + getStationaryRobotAngleOffsetMultiplier(distanceToSpeaker) * (distanceToSpeaker) - robotSpeedAdjustementFunction(distanceToSpeaker) * (dDistanceToSpeaker / distanceToSpeaker);
    return theta;
  }

  public double getRequiredShooterAngleFromTable(Double dis) {
    return angleMap.get(dis);
  }

  public double getStationaryRobotAngleOffsetMultiplier(Double distanceToSpeaker) {
    return 0.0134 + 7.4E-04 * distanceToSpeaker + -7.8E-05 * Math.pow(distanceToSpeaker, 2); // POLYNOMIAL
    // return 2.81E-04*distanceToSpeaker + 0.0137; //LINEAR
    // return 0.0101;//CONSTANT
  }

  // private double robotSpeedAdjustementFunction(Double distanceToSpeaker) {
  //   var x = distanceToSpeaker; 
  //   return -0.0188 
  //     + (0.117 * x) 
  //     - (0.0465 * Math.pow(x, 2)) 
  //     + (7.75e-3 * Math.pow(x, 3)) 
  //     - (4.9e-4 * Math.pow(x, 4)) 
  //     ;
  // } //Second iteration

  private double robotSpeedAdjustementFunction(Double distanceToSpeaker) {
    var x = distanceToSpeaker; 
    return 0.0108
     + 0.0355*x 
     + -4.33E-03*Math.pow(x,2) 
     + -1.69E-03*Math.pow(x,3) 
     + 5.12E-04*Math.pow(x,4) 
     + -5.08E-05*Math.pow(x,5) 
     + 1.73E-06*Math.pow(x,6)
     ;
  } // third iteration

  // private double robotSpeedAdjustementFunction(Double distanceToSpeaker) {
  //   var x = distanceToSpeaker; 
  //   return -0.0109
  //    + 0.0739*x + 
  //    -0.0245*Math.pow(x,2) 
  //    + 4.49E-03*Math.pow(x,3) 
  //    + -4.93E-04*Math.pow(x,4) 
  //    + 2.95E-05*Math.pow(x,5) 
  //    + -7.36E-07*Math.pow(x,6)
  //    ;
  // } // first iteration

  public double getShooterAngle() {
    var angle = (m_angleEncoder.getAbsolutePosition()*360 - kAngleEncoderOffset) % 360;
    angle = -angle;
    return angle < -180 ? angle + 360: angle;
  }

}
