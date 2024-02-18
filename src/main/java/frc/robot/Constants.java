// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.MeasurementConstants.kFieldX;
import static frc.robot.Constants.MeasurementConstants.kInchesToMeters;

import java.util.HashMap;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.COTSTalonFXSwerveConstants;

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

  public static final class LEDs {
    public static final double kConfidentShotRange = 4.;
    public static final double kMaxShotRange = 9.;
  }

  public static final class Shooter {
    public static final double kShootingAdjustmentMultiplier = 1.5;
    public static final double kStationaryRobotAngleMultiplier = 0.048; // 0.0222 // 0.015 // 0.0101
    public static final double kAngleEncoderOffset = 14.27+68.5;
    public static final double kMaxSpeedRPM = 5676;
    public static final double kMotorToAngleGearRatio = 125 * 42 / 18;

    public static final double kAmpAngle = -25.;
    public static final double kTransferAngle = 15.; //DEGREES - ANGLE FOR INTAKE TO SHOOTER TRANSFER

    public static final class PID {
      public static final double kAngleP = 0.02;
      public static final double kAngleI = 0.0;
      public static final double kAngleD = 0.0;

    }
  }

  public static final class Intake {
    public static final double kIntakeEncoderOffset = 0.25;
    public static final double kEncoderGearRatio = 0.5;
    public static final double kMinRotation = 0.0;  // DEGREES
    public static final double kMaxRotation = 70.0; // DEGREES

    public static final class PID {
      public static final double kIAngleP = 0.005;
      public static final double kIAngleI = 0.;
      public static final double kIAngleD = 0.;
    }
  }

  public static final class Elevator {
    public static final double kMotorRotationsToMeters = 0.008; // 8 mm per revolution
    public static final double kMaxHeight = 0.37;
    public static final double kMinHeight = 0.03;

    public static final double kStallPower = 0.02;

    public static final double kTransferHeight = 0.03;

    public static final class PID {
      public static final double kElevatorP = 0.1;
      public static final double kElevatorI = 0.;
      public static final double kElevatorD = 0.;
    }
  }

  public static final class MeasurementConstants {
    // This is based on the CAD model (divided by two to represent distance from center of robot) 
    public static final double kInchesToMeters = 39.37;
    public static final double kModuleXOffsetMeters = 20.75 / kInchesToMeters / 2; //TODO FIX THIS
    public static final double kModuleYOffsetMeters = 20.75 / kInchesToMeters / 2; //TODO FIX THIS
    public static final double kDiagModuleOffsetMeters = Math.sqrt(Math.pow(kModuleXOffsetMeters, 2) + Math.pow(kModuleYOffsetMeters, 2));
    public static final double kWheelDiameterMeters = 0.10033; // 4 inches - diameter of the wheels == 0.10033

    public static final double kFieldX = 649 / kInchesToMeters;
    public static final double kFieldY = 319 / kInchesToMeters;

    public static final double kFrontLeftEncoderOffset = 0.0;
    public static final double kBackLeftEncoderOffset = 0.0;
    public static final double kFrontRightEncoderOffset = 0.0;
    public static final double kBackRightEncoderOffset = 0.0; 

    public static final double kMaxSpeedMetersPerSecond = 5880 / 60.0 *
      Swerve.kDriveReduction *
      MeasurementConstants.kWheelDiameterMeters * Math.PI; // ~ 5.2 m/s
      public static final double kMaxAccelerationMetersPerSecondSquared = 3.55; //3
    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond /
      Math.hypot(kModuleXOffsetMeters / 2.0, kModuleYOffsetMeters / 2.0);
  


    public static final double kSpeakerOpeningMinHeight = 1.98; //METERS      
    public static final double kSpeakerOpeningMaxHeight = 2.11; //METERS  
    public static final double kSpeakerHoodDepth = 0.46; //METERS    
    public static final double shooterReleaseHeight = 0.5334; //0.5334 METERS = 21 INCHES, 0.9144 METERS = 36 INCHES, 0.762 METERS = 30 INCHES
  
    public static final Translation2d kRedSpeakerCoords = new Translation2d(16.579342, 5.547868);  // POSITION OF THE SPEAKER ON THE RED FIELD (METERS)
    public static final Translation2d kBlueSpeakerCoords = new Translation2d(-0.0381, 5.547868); // POSITION OF THE SPEAKER ON THE BLUE FIELD (METERS)
  
  
    }

  public static final class Swerve {
    public static final double kSpeedMultiplier = 1; // limits robot speed
    public static final double kRotationSpeedMultiplier = 0.6;
    public static final double kDriveDeadband = 0.05;
    public static final double kAutoDriveSpeedLimiter = 1.;

    public static final double kMaxVoltage = 13.0;
    public static final double kAccelerationSeconds = 0.15; // 0.5 seconds to reach full speed

    public static final int kDriveMotorCurrentLimit = 40;
    public static final int kSteerMotorCurrentLimit = 20;

    public static final double kDriveReduction = 1 / CTREConfigs.driveGearRatio; // Wheel revolutions per motor revolution`
    public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0); // Module revolutions per motor revolution

    public static final double kDriveEncoderPositionConversionFactor = Math.PI * MeasurementConstants.kWheelDiameterMeters * kDriveReduction;
    public static final double kSteerEncoderPositionConversionFactor = 360 * kSteerReduction; 

    public static final double kWheelCircumference = MeasurementConstants.kWheelDiameterMeters * Math.PI;

    public static final double driveKS = 0.054977; //SYSID CALCULATED 0.054977
    public static final double driveKV = 12.881; //12.881
    public static final double driveKA = 1.1502; //1.1502

    public static final class CTREConfigs {
      public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

      public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

      public static final double peakVoltage = 13.5;

      public static final int driveCurrentLimit = 35;
      public static final int driveCurrentThreshold = 60;
      public static final double driveCurrentThresholdTime = 0.1;
      public static final boolean driveEnableCurrentLimit = true;

      public static final double openLoopRamp = 0.;
      public static final double closedLoopRamp = .1;

      public static final double driveGearRatio = (5.9 / 1.0);//L2 default is 6.75 //5.9 to 1 with new pinions

      public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

      public static final double driveKP = 0.16;
      public static final double driveKI = 0.0015; // Used in module control TODO: Tune
      public static final double driveKD = 0.0;
    }
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

      public static final double kTurnP = 0.02;// 0.075
      public static final double kTurnI = 0.0002; // Used in pose control
      public static final double kTurnD = 0.00; //0.0075
      
      public static final double kLLTurnP = 0.01;// 0.075
      public static final double kLLTurnI = 0.0002; // Used in limelight pose control
      public static final double kLLTurnD = 0.0; //0.0075

      public static final double kTiltP = 0.01;
      public static final double kTiltI = 0.0;
      public static final double kTiltD = 0.0;

      public static final double kDriveTolerance = 0.01;
      public static final double kTurnTolerance = 1.0;
    }
    
  }

  public static final class CANConstants {
    public static final String CANivoreID = "CANivore 1";

    public static final int kFrontLeftDriveMotorID = 3;
    public static final int kBackLeftDriveMotorID = 1;
    public static final int kFrontRightDriveMotorID = 2;
    public static final int kBackRightDriveMotorID = 4;

    public static final int kFrontLeftSteerMotorID = 7;
    public static final int kBackLeftSteerMotorID = 5;
    public static final int kFrontRightSteerMotorID = 6;
    public static final int kBackRightSteerMotorID = 8;

    public static final int kFrontLeftEncoderID = 11;
    public static final int kBackLeftEncoderID = 9;
    public static final int kFrontRightEncoderID = 10;
    public static final int kBackRightEncoderID = 12;


    public static final double kEncoderResolution = 4096;
    public static final double kEncoderDistancePerPulse =
        (MeasurementConstants.kWheelDiameterMeters * Math.PI) / kEncoderResolution;
  }

  public static HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();
  public static final Pose2d RedAmpAlignment = new Pose2d(14.8, 7.3, new Rotation2d(-Math.PI/2));
  public static final class PathfindingPoints {
    public static final double kGridSize = 0.3;
    public static final class Red {
      public static final Pose2d Source = new Pose2d(8 * kGridSize, 4 * kGridSize, new Rotation2d(Math.PI/4 * 3));
      public static final Pose2d Amp = new Pose2d(14.675, 7.8, new Rotation2d(-Math.PI/2));
      public static final Pose2d Speaker = new Pose2d(kFieldX - 8 * kGridSize, 20 * kGridSize, new Rotation2d(0));
      public static final Pose2d CenterStage = new Pose2d(35 * kGridSize, 14 * kGridSize, new Rotation2d(Math.PI));
      public static final Pose2d StageLeft = new Pose2d(40 * kGridSize, 11 * kGridSize, new Rotation2d(Math.PI / 4 * 3));
      public static final Pose2d StageRight = new Pose2d(40 * kGridSize, 16.5 * kGridSize, new Rotation2d(-Math.PI / 4 * 3));
      
    }
    public static final class Blue {
      public static final Pose2d Source = new Pose2d(kFieldX - Red.Source.getX(), Red.Source.getY(), new Rotation2d(Math.PI/4));
      public static final Pose2d Amp = new Pose2d(kFieldX - Red.Amp.getX(), Red.Amp.getY(), new Rotation2d(Math.PI/2));
      public static final Pose2d Speaker = new Pose2d(kFieldX - Red.Speaker.getX(), Red.Speaker.getY(), new Rotation2d(Math.PI));
      public static final Pose2d CenterStage = new Pose2d(kFieldX - Red.CenterStage.getX(), Red.CenterStage.getY(), new Rotation2d(0));
      public static final Pose2d StageLeft = new Pose2d(kFieldX - Red.StageLeft.getX(), Red.StageLeft.getY(), new Rotation2d(-Math.PI/4));
      public static final Pose2d StageRight = new Pose2d(kFieldX - Red.StageRight.getX(), Red.StageRight.getY(), new Rotation2d(Math.PI/4));
    }
  }
  public static final class AprilTags {
    public static final Pose3d ID1  = new Pose3d(593.68 / kInchesToMeters,   9.68 / kInchesToMeters, 53.38 / kInchesToMeters, new Rotation3d(0.0 , 0.0, 120 / 180 * Math.PI));
    public static final Pose3d ID2  = new Pose3d(637.21 / kInchesToMeters,  34.79 / kInchesToMeters, 53.38 / kInchesToMeters, new Rotation3d(0.0 , 0.0, 120 / 180 * Math.PI));
    public static final Pose3d ID3  = new Pose3d(652.73 / kInchesToMeters, 196.17 / kInchesToMeters, 57.13 / kInchesToMeters, new Rotation3d(0.0 , 0.0, 180 / 180 * Math.PI));
    public static final Pose3d ID4  = new Pose3d(652.73 / kInchesToMeters, 218.42 / kInchesToMeters, 57.13 / kInchesToMeters, new Rotation3d(0.0 , 0.0, 180 / 180 * Math.PI));
    public static final Pose3d ID5  = new Pose3d(578.77 / kInchesToMeters, 323.00 / kInchesToMeters, 53.38 / kInchesToMeters, new Rotation3d(0.0 , 0.0, 270 / 180 * Math.PI));
    public static final Pose3d ID6  = new Pose3d( 72.50 / kInchesToMeters, 323.00 / kInchesToMeters, 53.38 / kInchesToMeters, new Rotation3d(0.0 , 0.0, 270 / 180 * Math.PI));
    public static final Pose3d ID7  = new Pose3d( -1.50 / kInchesToMeters, 218.42 / kInchesToMeters, 57.13 / kInchesToMeters, new Rotation3d(0.0 , 0.0,   0 / 180 * Math.PI));
    public static final Pose3d ID8  = new Pose3d( -1.50 / kInchesToMeters, 196.17 / kInchesToMeters, 57.13 / kInchesToMeters, new Rotation3d(0.0 , 0.0,   0 / 180 * Math.PI));
    public static final Pose3d ID9  = new Pose3d( 14.02 / kInchesToMeters,  34.79 / kInchesToMeters, 53.38 / kInchesToMeters, new Rotation3d(0.0 , 0.0,  60 / 180 * Math.PI));
    public static final Pose3d ID10 = new Pose3d( 57.54 / kInchesToMeters,   9.68 / kInchesToMeters, 53.38 / kInchesToMeters, new Rotation3d(0.0 , 0.0,  60 / 180 * Math.PI));
    public static final Pose3d ID11 = new Pose3d(468.69 / kInchesToMeters, 146.19 / kInchesToMeters, 52.00 / kInchesToMeters, new Rotation3d(0.0 , 0.0, 300 / 180 * Math.PI));
    public static final Pose3d ID12 = new Pose3d(468.69 / kInchesToMeters, 177.10 / kInchesToMeters, 52.00 / kInchesToMeters, new Rotation3d(0.0 , 0.0,  60 / 180 * Math.PI));
    public static final Pose3d ID13 = new Pose3d(441.74 / kInchesToMeters, 161.62 / kInchesToMeters, 52.00 / kInchesToMeters, new Rotation3d(0.0 , 0.0, 180 / 180 * Math.PI));
    public static final Pose3d ID14 = new Pose3d(209.48 / kInchesToMeters, 161.62 / kInchesToMeters, 52.00 / kInchesToMeters, new Rotation3d(0.0 , 0.0,   0 / 180 * Math.PI));
    public static final Pose3d ID15 = new Pose3d(182.73 / kInchesToMeters, 177.10 / kInchesToMeters, 52.00 / kInchesToMeters, new Rotation3d(0.0 , 0.0, 120 / 180 * Math.PI));
    public static final Pose3d ID16 = new Pose3d(182.73 / kInchesToMeters, 146.19 / kInchesToMeters, 52.00 / kInchesToMeters, new Rotation3d(0.0 , 0.0, 240 / 180 * Math.PI));
  }
}