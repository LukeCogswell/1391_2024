// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.MeasurementConstants.*;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getRequiredShooterAngle(double distanceToSpeaker) {
    var theta2 = Math.atan( (kSpeakerOpeningMinHeight-shooterReleaseHeight) / distanceToSpeaker);
    var theta3 = Math.atan( (kSpeakerOpeningMaxHeight-shooterReleaseHeight) / (distanceToSpeaker - kSpeakerHoodDepth));
    var theta1 = 0.5 * (theta3-theta2);
    return theta1;
  }

}
