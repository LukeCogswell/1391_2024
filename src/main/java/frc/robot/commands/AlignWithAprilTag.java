// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class AlignWithAprilTag extends Command {
  private Drivetrain m_drivetrain;
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
  private PIDController xController = new PIDController(5., 0., 0.);
  private PIDController zController = new PIDController(5., 0., 0.);
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
    zController.setSetpoint(dis);
    zController.setTolerance(0.1);
    turnController.setSetpoint(0.0);
    turnController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var botPose = m_drivetrain.getBotPoseTagSpace();
    var xSpeed = xController.calculate(botPose[0]);
    var ySpeed = zController.calculate(botPose[2]);
    var rot = turnController.calculate(botPose[5]);

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
      return xController.atSetpoint() && zController.atSetpoint() && turnController.atSetpoint();
    }
    return true;
  }
}
