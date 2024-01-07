// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.MeasurementConstants.kInchesToMeters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String kRotationUnits = "degrees";
  public static final String kDistanceUnits = "meters";

  public static final class OIConstants {
    public static final int kDriverControllerID = 0;
    public static final int kAButtonID = 1;
    public static final int kBButtonID = 2;
    public static final int kXButtonID = 3;
    public static final int kYButtonID = 4;
    public static int kDriverControllerPort = 0;
    public static int kOperatorControllerPort = 1;

  }

  public static final class MeasurementConstants {
    // This is based on the CAD model (divided by two to represent distance from center of robot) 
    public static final double kInchesToMeters = 39.37;
    public static final double kModuleXOffsetMeters = 24.75 / kInchesToMeters / 2; // 21.5 inches - distance between left and right wheels
    public static final double kModuleYOffsetMeters = 24.75 / kInchesToMeters / 2; // 18.5 inches - distance between front and back wheels
    public static final double kDiagModuleOffsetMeters = Math.sqrt(Math.pow(kModuleXOffsetMeters, 2) + Math.pow(kModuleYOffsetMeters, 2));
    public static final double kWheelDiameterMeters = 0.10033; // 4 inches - diameter of the wheels

    public static final double kFieldX = 649 / kInchesToMeters;
    public static final double kFieldY = 319 / kInchesToMeters;

    public static final double kFrontLeftEncoderOffset = 323.17;
    public static final double kBackLeftEncoderOffset = 260.33;
    public static final double kFrontRightEncoderOffset = 149.24;
    public static final double kBackRightEncoderOffset = 0.0; 

    public static final double kMaxSpeedMetersPerSecond = 5880 / 60.0 *
      SwerveModuleConstants.kDriveReduction *
      MeasurementConstants.kWheelDiameterMeters * Math.PI; // ~ 4.6 m/s
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond /
      Math.hypot(kModuleXOffsetMeters / 2.0, kModuleYOffsetMeters / 2.0);
  }

  public static final class SwerveModuleConstants {
    public static final double kSpeedMultiplier = 1; // limits robot speed
    public static final double kRotationSpeedMultiplier = 0.7;
    public static final double kDriveDeadband = 0.05;

    public static final double kMaxVoltage = 12.0;
    public static final double kAccelerationSeconds = 0.15; // 0.5 seconds to reach full speed

    public static final int kDriveMotorCurrentLimit = 40;
    public static final int kSteerMotorCurrentLimit = 20;

    public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // Wheel revolutions per motor revolution`
    public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0); // Module revolutions per motor revolution

    public static final double kDriveEncoderPositionConversionFactor = Math.PI * MeasurementConstants.kWheelDiameterMeters * kDriveReduction;
    public static final double kSteerEncoderPositionConversionFactor = 360 * kSteerReduction; 

    public static final class PID {
      public static final double kSteerP = 0.01;
      public static final double kSteerI = 0.0; // Used in module control
      public static final double kSteerD = 0.0;

      public static final double kModDriveP = 0.16;
      public static final double kModDriveI = 0.0015; // Used in module control TODO: Tune post-competition
      public static final double kModDriveD = 0.0;

      public static final double kDriveP = 5.0;
      public static final double kDriveI = 0.0; // Used in pose control
      public static final double kDriveD = 0.0;

      public static final double kTurnP = 1.15;
      public static final double kTurnI = 0.0; // Used in pose control
      public static final double kTurnD = 0.0;

      public static final double kTiltP = 0.01;
      public static final double kTiltI = 0.0;
      public static final double kTiltD = 0.0;

      public static final double kDriveTolerance = 0.01;
      public static final double kTurnTolerance = 1.0;
    }
    
  }

  public static final class CANConstants {
    public static final int kFrontLeftDriveMotorID = 1;
    public static final int kBackLeftDriveMotorID = 3;
    public static final int kFrontRightDriveMotorID = 2;
    public static final int kBackRightDriveMotorID = 4;

    public static final int kFrontLeftSteerMotorID = 5;
    public static final int kBackLeftSteerMotorID = 7;
    public static final int kFrontRightSteerMotorID = 6;
    public static final int kBackRightSteerMotorID = 8;

    public static final int kFrontLeftEncoderID = 9;
    public static final int kBackLeftEncoderID = 11;
    public static final int kFrontRightEncoderID = 10;
    public static final int kBackRightEncoderID = 12;


    public static final double kEncoderResolution = 4096;
    public static final double kEncoderDistancePerPulse =
        (MeasurementConstants.kWheelDiameterMeters * Math.PI) / kEncoderResolution;
  }

  public static HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();
}