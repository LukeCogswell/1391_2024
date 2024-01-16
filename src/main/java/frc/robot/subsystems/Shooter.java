// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Shooter.kStationaryRobotAngleMultiplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m_angleMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax m_loaderMotor = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax m_shooterMotorTop = new CANSparkMax(13, MotorType.kBrushless); 
  private final CANSparkMax m_shooterMotorBottom = new CANSparkMax(14, MotorType.kBrushless);
  
  private DutyCycleEncoder m_angleEncoder = new DutyCycleEncoder(0);

  /** Creates a new Shooter. */
  public Shooter() {
    m_shooterMotorTop.setInverted(false);
    m_shooterMotorBottom.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLoaderMotor(Double power) {
    m_loaderMotor.set(power);
  }

  public void setShooterSpeed(Double speed) {
    m_shooterMotorTop.set(speed/kMaxSpeedRPM);
    m_shooterMotorBottom.set(speed/kMaxSpeedRPM);
    
  }
  
  public void stopShooter() {
    m_shooterMotorTop.set(0);
    m_shooterMotorBottom.set(0);

  }

  public void setTopShooterSpeed(Double speed) {
    m_shooterMotorTop.set(speed/kMaxSpeedRPM);
  }
  
  public void setBottomShooterSpeed(Double speed) {
    m_shooterMotorBottom.set(speed/kMaxSpeedRPM);

  }

  public double getTopShooterSpeed() {
    return m_shooterMotorTop.getEncoder().getVelocity();  
  }
  
  public double getBottomShooterSpeed() {
    return m_shooterMotorBottom.getEncoder().getVelocity();  
  }


  public void setAngleMotor(Double power) {
    m_angleMotor.set(power);
  }

  public double getShooterAngle() {
    var angle = (m_angleEncoder.getAbsolutePosition()*360 - kAngleEncoderOffset - 180) % 360 + 180;
    return angle > 180 ? angle - 360: angle;
  }


  public double getRequiredShooterAngle(Double distanceToSpeaker, Double dDistanceToSpeaker) {
    var theta2 = Math.atan( (kSpeakerOpeningMinHeight-shooterReleaseHeight) / distanceToSpeaker);
    var theta3 = Math.atan( (kSpeakerOpeningMaxHeight-shooterReleaseHeight) / (distanceToSpeaker - kSpeakerHoodDepth));
    var theta1 = 0.5 * (theta3+theta2);
    var theta = theta1 + kStationaryRobotAngleMultiplier * (distanceToSpeaker) - robotSpeedAdjustementFunction(distanceToSpeaker) * (dDistanceToSpeaker / distanceToSpeaker);
    return theta;
  }

  private double robotSpeedAdjustementFunction(Double distanceToSpeaker) {
    var x = distanceToSpeaker; 
    return -0.0109 
      + (0.0739 * x) 
      - (0.0245 * Math.pow(x, 2)) 
      + (4.49e-3 * Math.pow(x, 3)) 
      - (4.93e-4 * Math.pow(x, 4)) 
      + (2.95e-5 * Math.pow(x, 5)) 
      - (7.36e-7 * Math.pow(x, 6));
  }

  // public double getChangeInShooterAngle(Double distanceToSpeaker, Double dDistanceToSpeaker) {
  //   var d = distanceToSpeaker;
  //   var dd = dDistanceToSpeaker;
  //   var dThetaV = 0.5 * 
  //     (
  //       ( 1 / (1 + Math.pow((2.11 - shooterReleaseHeight) / (d-0.46), 2)) 
  //         * 
  //         ( (shooterReleaseHeight - 2.11) / (Math.pow(d-0.46, 2)) ) 
  //         * 
  //         dd)
  //        + 
  //       (( 1 / (1 + Math.pow((1.98-shooterReleaseHeight) / (d), 2) ) )
  //         * 
  //         ( (shooterReleaseHeight-1.98) / (Math.pow(d, 2)) ) 
  //         * 
  //         dd));
  //   return dThetaV * 180 / Math.PI;
  // }


}
