// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.MeasurementConstants.*;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final CANSparkMax m_angleMotor = new CANSparkMax(11, MotorType.kBrushless);
  private DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(0);

  public Turret() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle", getShooterAngle());
    // This method will be called once per scheduler run
  }

  public void setAngleMotor(Double power) {
    if (getShooterAngle() <= -15 || getShooterAngle() >= 80) {
      m_angleMotor.set(0.);

    } else {
      m_angleMotor.set(power);
    }
  }

  public double getRequiredShooterAngle(Double distanceToSpeaker, Double dDistanceToSpeaker) {
    var theta1 = Math.atan((((kSpeakerOpeningMaxHeight + kSpeakerOpeningMinHeight)/2) - shooterReleaseHeight) / (distanceToSpeaker-0.23));
    var theta = theta1 + getStationaryRobotAngleOffsetMultiplier(distanceToSpeaker) * (distanceToSpeaker) - robotSpeedAdjustementFunction(distanceToSpeaker) * (dDistanceToSpeaker / distanceToSpeaker);
    return theta;
  }

  public double getStationaryRobotAngleOffsetMultiplier(Double distanceToSpeaker) {
    if (distanceToSpeaker >= 4.2) {
      return -3.23e-3 * distanceToSpeaker + 0.0635;
    } else {
      return -0.0208 * distanceToSpeaker + 0.138;
    }
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
    return -0.0109
     + 0.0739*x + 
     -0.0245*Math.pow(x,2) 
     + 4.49E-03*Math.pow(x,3) 
     + -4.93E-04*Math.pow(x,4) 
     + 2.95E-05*Math.pow(x,5) 
     + -7.36E-07*Math.pow(x,6)
     ;
  } // first iteration

  public double getShooterAngle() {
    var angle = 180 - (m_angleEncoder.getAbsolutePosition()*360 - kAngleEncoderOffset - 180) % 360;
    return angle > 180 ? angle - 360: angle;
  }

}
