// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.Swerve.PID.*;

import java.util.Arrays;

public class AlignWithAprilTag extends Command {
  private Drivetrain m_drivetrain;
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
  private PIDController xController = new PIDController(1., 0., 0.);
  private PIDController zController = new PIDController(1., 0., 0.);
  private Double dis;
  private Double maxSpeed = 1.;
  /** Creates a new AlignWithAprilTag. */
  public AlignWithAprilTag(Drivetrain drivetrain, Double distance) {
    m_drivetrain = drivetrain;
    dis = distance;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(0.);
    xController.setTolerance(0.1);
    zController.setSetpoint(-dis);
    zController.setTolerance(0.1);
    if (m_drivetrain.getTID() == 5 || m_drivetrain.getTID() == 6) {
      turnController.setSetpoint(-90.0);
    } else if (m_drivetrain.getTID() == 1 || m_drivetrain.getTID() == 2) {
      turnController.setSetpoint(180-38.6);
    } else if (m_drivetrain.getTID() == 9 || m_drivetrain.getTID() == 10) {
      turnController.setSetpoint(38.6);
    }
    turnController.setTolerance(0.5);
    turnController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var botPose = m_drivetrain.getBotPoseTagSpace();
    var ySpeed = xController.calculate(botPose[0]);
    var xSpeed = -zController.calculate(botPose[2]);
    var rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees());

    xSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);

    m_drivetrain.drive(xSpeed, ySpeed, rot, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_drivetrain.getTV()) {
      if (!Arrays.asList(1., 2., 5., 6., 9., 10.).contains(m_drivetrain.getTID())) { 
        return true;
      }
      return xController.atSetpoint() && zController.atSetpoint() && turnController.atSetpoint();
    }
    return true;
  }
}
