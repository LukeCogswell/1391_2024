// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Shooter.RangeTable.*;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.CANConstants.*;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final CANSparkMax m_angleMotor = new CANSparkMax(kTurretMotorID, MotorType.kBrushless);
  private DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(0);

  public Turret() {
    m_angleMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle", getShooterAngle());
    // This method will be called once per scheduler run
  }

  public void setAngleMotor(Double power) {
    m_angleMotor.set(power);
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

  public double getRequiredShooterAngleFromTable(Double dis, Double dDistanceToSpeaker) {
    var initialAngle = 0.;
    if (dis <= entry0[0]) {
      initialAngle = entry0[1];
    } else if (dis <= entry1[0]) {
      initialAngle = MathUtil.interpolate(entry0[1], entry1[1], (dis - entry0[0]) / (entry1[0] - entry0[0]));
    } else if (dis <= entry2[0]) {
      initialAngle = MathUtil.interpolate(entry1[1], entry2[1], (dis - entry1[0]) / (entry2[0] - entry1[0]));
    } else if (dis <= entry3[0]) {
      initialAngle = MathUtil.interpolate(entry2[1], entry3[1], (dis - entry2[0]) / (entry3[0] - entry2[0]));
    } else if (dis <= entry4[0]) {
      initialAngle = MathUtil.interpolate(entry3[1], entry4[1], (dis - entry3[0]) / (entry4[0] - entry3[0]));
    } else if (dis <= entry5[0]) {
      initialAngle = MathUtil.interpolate(entry4[1], entry5[1], (dis - entry4[0]) / (entry5[0] - entry4[0]));
    } else if (dis <= entry6[0]) {
      initialAngle = MathUtil.interpolate(entry5[1], entry6[1], (dis - entry5[0]) / (entry6[0] - entry5[0]));
    } else if (dis <= entry7[0]) {
      initialAngle = MathUtil.interpolate(entry6[1], entry7[1], (dis - entry6[0]) / (entry7[0] - entry6[0]));
    } else if (dis <= entry8[0]) {
      initialAngle = MathUtil.interpolate(entry7[1], entry8[1], (dis - entry7[0]) / (entry8[0] - entry7[0]));
    } else if (dis <= entry9[0]) {
      initialAngle = MathUtil.interpolate(entry8[1], entry9[1], (dis - entry8[0]) / (entry9[0] - entry8[0]));
    } else if (dis <= entry10[0]) {
      initialAngle = MathUtil.interpolate(entry9[1], entry10[1], (dis - entry9[0]) / (entry10[0] - entry9[0]));
    } else if (dis <= entry11[0]) {
      initialAngle = MathUtil.interpolate(entry10[1], entry11[1], (dis - entry10[0]) / (entry11[0] - entry10[0]));
    } else if (dis > entry11[0]) {
      initialAngle = entry11[1];
    }
    return initialAngle * Math.PI / 180;
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
    var angle = 180 - (m_angleEncoder.getAbsolutePosition()*360 - kAngleEncoderOffset - 180) % 360;
    angle = -angle;
    return angle < -180 ? angle + 360: angle;
  }

}
