// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import static frc.robot.Constants.Swerve.CTREConfigs.*;

/** Add your docs here. */
public final class CTREConfigs {
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    
    public CTREConfigs() {
      /** Swerve Drive Motor Configuration */

        swerveDriveFXConfig.Voltage.PeakForwardVoltage = peakVoltage;
        swerveDriveFXConfig.Voltage.PeakReverseVoltage = -peakVoltage;

        swerveDriveFXConfig.Audio.AllowMusicDurDisable = false;

        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = driveCurrentThresholdTime;
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = driveEnableStatorCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = driveStatorCurrentLimit;
        
        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = driveKP;
        swerveDriveFXConfig.Slot0.kI = driveKI;
        swerveDriveFXConfig.Slot0.kD = driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

    }
    
    public TalonFXConfiguration getSwerveDriveConfig() {
      return swerveDriveFXConfig;
    }
}
