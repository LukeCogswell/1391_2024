// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToPoint extends Command {
  private Drivetrain m_drivetrain;
  private PIDController xController = new PIDController(0.3, 0, 0);
  private PIDController yController = new PIDController(0.3, 0, 0);
  private PIDController rotController = new PIDController(0.02, 0, 0);
  private Pose2d pose;
  private Double kMaxRotSpeed = 1.;
  private Double kMaxSpeed = Constants.MeasurementConstants.kMaxSpeedMetersPerSecond;
  /** Creates a new DriveToPoint. */
  public DriveToPoint(Drivetrain drivetrain, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    pose = targetPose;
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(pose.getX());
    xController.setTolerance(0.1);
    yController.setSetpoint(pose.getY());
    yController.setTolerance(0.1);
    rotController.setSetpoint(pose.getRotation().getDegrees());
    rotController.setTolerance(3);
    rotController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var botPose = m_drivetrain.getFieldPosition();
    var xDrive = xController.calculate(botPose.getX());
    var yDrive = yController.calculate(botPose.getY());
    var rot = rotController.calculate(botPose.getRotation().getDegrees());

    xDrive = MathUtil.clamp(xDrive, -kMaxSpeed, kMaxSpeed);
    yDrive = MathUtil.clamp(yDrive, -kMaxSpeed, kMaxSpeed);
    rot = MathUtil.clamp(rot, -kMaxRotSpeed, kMaxRotSpeed);

    m_drivetrain.drive(xDrive, yDrive, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
    rotController.close();
    xController.close();
    yController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
  }
}
