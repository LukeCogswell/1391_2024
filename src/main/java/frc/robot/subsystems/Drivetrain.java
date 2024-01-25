// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MeasurementConstants.*;

import org.ejml.simple.SimpleMatrix;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import static frc.robot.Constants.CANConstants.*;

public class Drivetrain extends SubsystemBase {
  
  private NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");

  private SwerveModule m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  
  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(  kModuleXOffsetMeters, kModuleYOffsetMeters), // BACK RIGHT
    new Translation2d( kModuleXOffsetMeters,  -kModuleYOffsetMeters), // BACK LEFT
    new Translation2d(  -kModuleXOffsetMeters, kModuleYOffsetMeters),  // FRONT RIGHT
    new Translation2d( -kModuleXOffsetMeters, -kModuleYOffsetMeters)   // FRONT LEFT
  );

  private SwerveDrivePoseEstimator m_odometry;
  private AHRS m_navX = new AHRS();

  private Field2d field = new Field2d();

  public double desiredVelocityAverage = 0;
  public double actualVelocityAverage = 0;



  /** Creates a new Drivetrain. */
  public Drivetrain() {
    
    m_frontLeft = new SwerveModule(
      kFrontLeftDriveMotorID,
      kFrontLeftSteerMotorID,
      kFrontLeftEncoderID,
      kFrontLeftEncoderOffset
      );
      
      m_frontRight = new SwerveModule(
      kFrontRightDriveMotorID,
      kFrontRightSteerMotorID,
      kFrontRightEncoderID,
      kFrontRightEncoderOffset
      );
      
      m_backLeft = new SwerveModule(
      kBackLeftDriveMotorID,
      kBackLeftSteerMotorID,
      kBackLeftEncoderID,
      kBackLeftEncoderOffset
      );
      
      m_backRight = new SwerveModule(
        kBackRightDriveMotorID,
      kBackRightSteerMotorID,
      kBackRightEncoderID,
      kBackRightEncoderOffset
      );
      // m_navX.zeroYaw();
      
      m_odometry = new SwerveDrivePoseEstimator(m_kinematics, getGyroRotation2d(), getModulePositions(), new Pose2d());
      
      // Configure the AutoBuilder last
      AutoBuilder.configureHolonomic(
        this::getFieldPosition, // Robot pose supplier
        this::setFieldPosition, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        kMaxSpeedMetersPerSecond, // Max module speed, in m/s
        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
        new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {if (DriverStation.getAlliance().isPresent()) {
          return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;}
          return false;},
          this // Reference to this subsystem to set requirements
    );
    
    SmartDashboard.putBoolean("init-PRESENT?", DriverStation.getAlliance().isPresent());
    setFieldPosition(new Pose2d(new Translation2d(1, kFieldY/2), new Rotation2d(0.0)));
    if (DriverStation.getAlliance().isPresent()) {
      SmartDashboard.putBoolean("init-RED?", DriverStation.getAlliance().get() == Alliance.Red);
      setFieldPosition(new Pose2d(new Translation2d(kFieldX-1, kFieldY/2), new Rotation2d(Math.PI)));
    }

    
    setVisionStdDvs(1.0, 1.0, 9999.0);

    SmartDashboard.putData("Field", field);
  }
  
  @Override
  public void periodic() {
    updateOdometry();
    updateOdometryWithAprilTags();
    field.setRobotPose(getFieldPosition());
    SmartDashboard.putData("Field", field);
    // SmartDashboard.putString("Field Position", getFieldPosition().toString());
    // SmartDashboard.putNumber("Fused Heading", -m_navX.getFusedHeading());
    // if (getTV()) {
    //   SmartDashboard.putNumber("TagSpace Z", -getBotPoseTagSpace()[2]);
    //   SmartDashboard.putNumber("Gyro vs Limelight Diff", getFieldPosition().getRotation().getDegrees() -getBOTPOSE()[5]);
    // }
    // SmartDashboard.putBoolean("IS PRESENT?", DriverStation.getAlliance().isPresent());
    // if (DriverStation.getAlliance().isPresent()) {
    //   SmartDashboard.putBoolean("IS-RED?", DriverStation.getAlliance().get() == Alliance.Red);
    // }
    // This method will be called once per scheduler run
  }
  
  public void updateOdometryWithAprilTags() {
    if (getTV()){
      if (-getBotPoseTagSpace()[2] < 5.) {
        var botpose = getBOTPOSE();
        var botTagSpace = getBotPoseTagSpace();
        setVisionStdDvs(1.0 * -botTagSpace[2], 1.0 * -botTagSpace[2], 9999.0);
        Pose2d pos = new Pose2d(new Translation2d(botpose[0], botpose[1]), new Rotation2d(botpose[5]));
        m_odometry.addVisionMeasurement(pos, Timer.getFPGATimestamp() - (botpose[6]/1000.0)); 
      }
    }
  }

  public void setVisionStdDvs(Double x, Double y, Double theta) {
    SimpleMatrix StdDvs = new SimpleMatrix(3,1);
    StdDvs.set(0, 0, x);
    StdDvs.set(1, 0, y);
    StdDvs.set(2, 0, theta);
    m_odometry.setVisionMeasurementStdDevs(new Matrix<>(StdDvs));
  }

  public void setFieldPosition(Pose2d fieldPosition) {
    m_odometry.resetPosition(getGyroRotation2d(), getModulePositions(), fieldPosition);
  }

  public double getNavxYaw() { // returns the current yaw of the robot
    // var yaw = m_navX.getYaw() - Timer.getFPGATimestamp() * 0.1 / 36.8;
    var yaw = -m_navX.getFusedHeading();// - Timer.getFPGATimestamp() * 0.1 / 36.8;
    yaw = yaw > 180 ? yaw - 360 : yaw;
    yaw = yaw < -180 ? yaw + 360 : yaw;
    return yaw;
  }

  public double getNavxPitch() { // returns the current pitch of the robot
    return m_navX.getPitch();
  }

  public double getNavxRoll() { // returns the current roll of the robot
    return m_navX.getRoll();
  }

  public void zeroGyro() { // zeros the gyro
    m_navX.reset();
  }

  public Rotation2d getGyroRotation2d() { // returns the current rotation of the robot
    return Rotation2d.fromDegrees(-m_navX.getFusedHeading());
  }

  public double getTX() {
    return m_limelight.getEntry("tx").getDouble(0.0);
  }
  
  public double getTY() {
    return m_limelight.getEntry("ty").getDouble(0.0);
  }

  public boolean getTV() {
    return m_limelight.getEntry("tv").getDouble(0.0) == 1.0;
  }

  public double[] getBOTPOSE() {
    return m_limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[]{});
  }

  public double[] getBotPoseTagSpace() {
    return m_limelight.getEntry("botpose_targetspace").getDoubleArray(new double[]{});
  }
  
  public void updateOdometry() { // updates the odometry of the robot
    m_odometry.update(
      getGyroRotation2d(),
      getModulePositions()
    );
  }

  public SwerveModulePosition[] getModulePositions() { // returns the positions of the modules
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  public Pose2d getFieldPosition() { // returns the current position of the robot
    return m_odometry.getEstimatedPosition();
  }

  public void stop() { // stops the robot (module rotation and translation)
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public double getOdometryYaw() { // returns the current yaw of the robot
    var pos = -m_odometry.getEstimatedPosition().getRotation().getDegrees() % 360;
    return pos < -180 ? pos + 360 : pos;
  }

  public void drive(double xSpeed, double ySpeed, double rot) { // drives the robot (always field relative)
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed,
        ySpeed,
        rot,
        m_odometry.getEstimatedPosition().getRotation()
      )
    );

    setModuleStates(swerveModuleStates);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) { // drives the robot (field relative or robot relative)
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_odometry.getEstimatedPosition().getRotation())
        : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );

    setModuleStates(swerveModuleStates);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) { // drives the robot (field relative or robot relative)
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates);
  } 

  public void driveAroundPoint(double xSpeed, double ySpeed, double rot, boolean fieldRelative, Translation2d ctrOfRot) { // drives the robot (field relative or robot relative, variable center of rotation)
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_odometry.getEstimatedPosition().getRotation())
        : new ChassisSpeeds(xSpeed, ySpeed, rot),
      ctrOfRot
    );

    setModuleStates(swerveModuleStates);
  }
  
  public void setModuleStates(SwerveModuleState[] states) { // sets the states of the modules
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

    desiredVelocityAverage = Math.abs(states[0].speedMetersPerSecond + states[1].speedMetersPerSecond + states[2].speedMetersPerSecond + states[3].speedMetersPerSecond) / 4;

    m_frontLeft.setDesiredStateClosed(states[0]);
    m_frontRight.setDesiredStateClosed(states[1]);
    m_backLeft.setDesiredStateClosed(states[2]);
    m_backRight.setDesiredStateClosed(states[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getModuleState(),
        m_frontRight.getModuleState(),
        m_backLeft.getModuleState(),
        m_backRight.getModuleState()
    };
  }

  public void DEBUG_OutputAbsoluteEncoderReadings() {
    System.out.println("FL" + m_frontLeft.getAbsoluteAngle());
    System.out.println("FR" + m_frontRight.getAbsoluteAngle());
    System.out.println("BL" + m_backLeft.getAbsoluteAngle());
    System.out.println("BR" + m_backRight.getAbsoluteAngle());
  }

  public double getDistanceToSpeaker() {
    double speakerX, speakerY;
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        speakerX = kRedSpeakerCoords.getX();
        speakerY = kRedSpeakerCoords.getY();
      } else {
        speakerX = kBlueSpeakerCoords.getX();
        speakerY = kBlueSpeakerCoords.getY();
      }
      var robotPose = getFieldPosition();
      var robotX = robotPose.getX();
      var robotY = robotPose.getY();
      var xOffset = Math.abs(speakerX-robotX);
      var yOffset = Math.abs(speakerY-robotY);
      var distance = Math.sqrt( (xOffset*xOffset) + (yOffset*yOffset));
      return distance;

    } else {
      return 999.9;

    }

  }

  public double getChangeInDistanceToSpeaker(Double dx, Double dy) {
    var pos = getFieldPosition();
    var dd = (pos.getX() * dx + pos.getY() * dy) / getDistanceToSpeaker();
    return dd;
  }
  
  public double getAngleToSpeaker() {
    double speakerX, speakerY;
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        speakerX = kRedSpeakerCoords.getX();
        speakerY = kRedSpeakerCoords.getY();
      } else {
        speakerX = kBlueSpeakerCoords.getX();
        speakerY = kBlueSpeakerCoords.getY();
      }
      var robotPose = getFieldPosition();
      var robotX = robotPose.getX();
      var robotY = robotPose.getY();
      var xOffset = speakerX-robotX;
      var yOffset = speakerY-robotY;
      var angle = Math.atan(yOffset/xOffset) * 180 / Math.PI;
      // SmartDashboard.putNumber("Raw Angle", angle);
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        angle = 180 + angle;
        angle = angle > 180 ? angle - 360: angle;
        return angle;
      } else {
        return angle; 
      }
    } else {
      return 0.0;
    }
  }

  public double getChangeInAngleToSpeaker(Double dx, Double dy) {
    var pos = getFieldPosition();
    var dTheta = (1 / (1+Math.pow(pos.getX()/pos.getY(), 2))) * ((pos.getY()*dx - pos.getX()*-dy) / (Math.pow(pos.getY(), 2)));
    return dTheta * 180 / Math.PI; 
  }

  public Command getCommandToPathfindToPoint(Pose2d targetPose, Double goalEndVelocity) {
    PathConstraints constraints = new PathConstraints(
      kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared, 
      kMaxAngularSpeedRadiansPerSecond, Units.degreesToRadians(720));
      
      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        goalEndVelocity, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

    return pathfindingCommand;
  }

}
