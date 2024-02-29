// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Loader;

import static frc.robot.Constants.CANConstants.CANivoreID;
import static frc.robot.Constants.LEDs.*;

import com.ctre.phoenix6.CANBus;

public class LEDDefault extends Command {
  private LEDs m_leds;
  private Drivetrain m_drivetrain;
  private Intake m_intake;
  private Loader m_loader;
  Color allianceColor;
  /** Creates a new LEDDefault. */
  public LEDDefault(LEDs leds, Drivetrain drivetrain, Intake intake, Loader loader) {
    m_leds = leds;
    m_drivetrain = drivetrain;
    m_intake = intake;
    m_loader = loader;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      allianceColor = Color.kRed;
    } else {
      allianceColor = Color.kBlue;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.isDisabled()) {
      m_leds.setBottomThird(allianceColor);
      if (m_loader.hasNoteInShooter()) {
        m_leds.setMiddleThird(Color.kOrangeRed);
      } else {
        m_leds.setMiddleThird(Color.kBlack);
      }
      // TOP THIRD SIGNALS CAN BUS VISIBILITY OF DEVICES
      // if (CANBus.getStatus(CANivoreID).BusOffCount)
      // //TOP THIRD SET TO COLORS BASED ON ROTATION OF ROBOT
      // if (m_drivetrain.getTV()) {
      //   double botRot = m_drivetrain.getBOTPOSE()[5];
      //   boolean turnLeft = Math.signum(botRot) == 1;
      //   botRot = Math.abs(botRot);
      //   if (allianceColor == Color.kRed) {
      //     botRot -= 180;
      //   }
      //   if (botRot <= 0.5) {
      //     m_leds.setTopThird(Color.kWhite);
      //   } else {
      //     if (turnLeft) {
      //       m_leds.setTopThird(Color.kRed);
      //     } else {
      //       m_leds.setTopThird(Color.kGreen);
      //     }
      //   }
      // } else {
      //   m_leds.setTopThird(Color.kBlack);
      // }
    } else {
      if (m_loader.hasNoteInShooter()) {
        //IF WE HAVE A NOTE IN THE SHOOTER, SET MORE LEDS THE CLOSER WE ARE IN A CERTAIN RANGE, IF WE SEE AN APRILTAG USE WHITE INSTEAD OF GREEN
        var percent = m_drivetrain.getDistanceToSpeaker() <= kConfidentShotRange ? 1 :
          1 - ((m_drivetrain.getDistanceToSpeaker() - kConfidentShotRange) / (kMaxShotRange-kConfidentShotRange));
        percent = MathUtil.clamp(percent, 0., 1.);
        m_leds.setPercent(percent, m_drivetrain.getTV() ? Color.kWhite : Color.kGreen);
      } else {
        //IF BLOCK FOR BOTTOM HALF OF LEDS - INTAKE STATUS
        if (m_intake.hasNoteInIntake()) {
          m_leds.setBottomHalf(Color.kGreen);
        } else if (m_intake.getTV()) {
          m_leds.setBottomHalf(Color.kOrangeRed);
        } else {
          m_leds.setBottomHalf(Color.kBlack);
        }
        //IF BLOCK FOR TOP HALF OF LEDS - APRILTAG LIMELIGHT STATUS
        if (m_drivetrain.getTV()) {
          m_leds.setTopHalf(Color.kWhite);
        } else {
          m_leds.setTopHalf(Color.kBlack);
        }
      }
    }
    m_leds.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
