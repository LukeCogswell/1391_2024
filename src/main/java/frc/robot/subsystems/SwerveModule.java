// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;  
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Lib.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.CTREConfigs;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Swerve.PID.*;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.CANConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private SwerveModulePosition m_modulePosition;
  
  public final TalonFX m_driveMotor;
  private final CANSparkMax m_steerMotor;

  private final RelativeEncoder m_steerRelativeEncoder;
  private final CANcoder m_steerEncoder;

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  private VelocityVoltage driveVelocity = new VelocityVoltage(0.);

  private final SparkPIDController m_steerPIDController;

  private final double m_steerEncoderOffset;

  /**
   * Swerve Module class that contains the drive and steer motors, along with their encoders
   * <p>
   * IDs found in {@link CANConstants}
   * 
   * @param driveMotorID - CAN ID of the drive motor
   * @param steerMotorID - CAN ID of the steer motor
   * @param steerEncoderID - CAN ID of the encoder
   * @param steerEncoderOffset
   */
  public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderID, double steerEncoderOffset) {

    m_steerEncoderOffset = steerEncoderOffset;
    
    m_steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);
    
    m_steerRelativeEncoder = m_steerMotor.getEncoder();
    m_steerEncoder = new CANcoder(steerEncoderID, Constants.CANConstants.CANivoreID);
    
    /* Drive Motor Config */
    m_driveMotor = new TalonFX(driveMotorID, Constants.CANConstants.CANivoreID);
    m_driveMotor.getConfigurator().apply(new CTREConfigs().getSwerveDriveConfig());
    m_driveMotor.getConfigurator().setPosition(0.0);
    setMotorSettings(m_steerMotor, kSteerMotorCurrentLimit);


    m_steerRelativeEncoder.setPositionConversionFactor(kSteerEncoderPositionConversionFactor); // Gives degrees
    m_steerRelativeEncoder.setVelocityConversionFactor(kSteerEncoderPositionConversionFactor / 60.0); // Gives degrees per second
    recalibrateRelativeEncoder();

    m_modulePosition = new SwerveModulePosition();
  
    m_steerPIDController = m_steerMotor.getPIDController();
      m_steerPIDController.setP(kSteerP);
      m_steerPIDController.setI(kSteerI);
      m_steerPIDController.setD(kSteerD);
      m_steerPIDController.setPositionPIDWrappingEnabled(true);
      m_steerPIDController.setPositionPIDWrappingMaxInput(360);
      m_steerPIDController.setPositionPIDWrappingMinInput(0);

    m_steerMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber(("Module "+(m_driveMotor.getDeviceId())), getAbsoluteAngle());
  }

  private void setMotorSettings(CANSparkMax motor, int currentLimit) { // sets the settings of the motor(s)
    // motor.restoreFactoryDefaults(); // restores factory default settings in case something was changed
    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 45); // sets update rate of motor faults, applied output, and is follower value to 45 ms
    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20); // sets update rate of motor velocity, current, temperature, and voltage to 20ms
    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20); // sets update rate of motor position to every 20 ms
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake); // sets idle mode to brake
    motor.enableVoltageCompensation(kMaxVoltage);
    motor.setSmartCurrentLimit(currentLimit);
    motor.setInverted(true);
  }

  public void recalibrateRelativeEncoder(){ // recalibrates the relative encoder
    m_steerRelativeEncoder.setPosition(m_steerEncoder.getAbsolutePosition().getValueAsDouble() * 360); // sets relative encoder to absolute encoder's value
  }

  public void updatePosition(){ // updates the current position of the module
    m_modulePosition.angle = getSteerAngle();
    m_modulePosition.distanceMeters = getDriveDistance();
  }
  
  public SwerveModulePosition getPosition(){ // returns the current position of the module
    updatePosition();
    return m_modulePosition;
  }

  public double getVelocity() {
    return m_driveMotor.getVelocity().getValueAsDouble() * kDriveReduction * kWheelCircumference;
  }

  public Rotation2d getSteerAngle(){ // returns the angle the module is facing
    double angle = m_steerRelativeEncoder.getPosition() - m_steerEncoderOffset;
    return Rotation2d.fromDegrees(angle); 
  }
 
  public double getDriveDistance(){ // returns the distance the module has traveled - add drive reduction for SYSID
    return m_driveMotor.getRotorPosition().getValueAsDouble() * kWheelCircumference * kDriveReduction;
  }

  public SwerveModuleState getModuleState(){ // returns the current state of the module
    return new SwerveModuleState(m_driveMotor.get(), getSteerAngle());
  }

  public void setDesiredStateClosed(SwerveModuleState state) { // sets the desired state of the module (closed loop)
    state = SwerveModuleState.optimize(state, getSteerAngle());
    m_steerPIDController.setReference(state.angle.getDegrees() + m_steerEncoderOffset, CANSparkMax.ControlType.kPosition);
    // If the module is going to be stationary, set the drive motor to 0
    if (state.speedMetersPerSecond == 0) {
      m_driveMotor.set(0);
    } else {
        driveVelocity.Velocity = Conversions.MPSToRPS(state.speedMetersPerSecond, Constants.Swerve.kWheelCircumference);
        driveVelocity.FeedForward = driveFeedForward.calculate(state.speedMetersPerSecond);
        m_driveMotor.setControl(driveVelocity);
    }
  }

  public void setDesiredStateOpen(SwerveModuleState state) { // sets the desired state of the module (open loop)
    state = SwerveModuleState.optimize(state, getSteerAngle());
    m_steerPIDController.setReference(state.angle.getDegrees() + m_steerEncoderOffset, CANSparkMax.ControlType.kPosition);
    m_driveMotor.set(state.speedMetersPerSecond / kMaxSpeedMetersPerSecond);
  }

  public void stop(){ // stops the module
    m_driveMotor.set(0);
    m_steerMotor.set(0);
  }

  public void setVoltage(Double volts) {
    m_driveMotor.setVoltage(volts);
  }

  public double getVoltage() {
   return m_driveMotor.getMotorVoltage().getValueAsDouble();
  }

  public double getAbsoluteAngle() { // only used for calibration or debugging
    return m_steerEncoder.getAbsolutePosition().getValueAsDouble() * 360; 
  }

}
