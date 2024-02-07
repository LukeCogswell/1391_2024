// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import static frc.robot.Constants.Swerve.PID.*;

public class EjectNotes extends Command {
  private Drivetrain m_drivetrain;
  private Shooter m_shooter;
  private Intake m_intake;
  private Loader m_loader;
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);

  /** Creates a new EjectNotes. */
  public EjectNotes(Drivetrain drivetrain, Shooter shooter, Intake intake, Loader loader) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_loader = loader;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, shooter, intake, loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      turnController.setSetpoint(-135.);
    } else {
      turnController.setSetpoint(45.);
    }
    m_shooter.setShooterSpeed(2500.); 
    m_intake.setIntake(1.);
    m_loader.setLoaderMotor(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(0., -2, turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterSpeed(0.);
    m_loader.setLoaderMotor(0.);
    m_intake.setIntake(0.);
    m_drivetrain.stop();
    turnController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
