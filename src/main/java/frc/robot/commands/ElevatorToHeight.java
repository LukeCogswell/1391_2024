// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Elevator.kStallPower;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorToHeight extends Command {
  private PIDController heightController = new PIDController(.5, 0.0, 0);
  private Double heightSetpoint;
  private Elevator m_elevator;

  /** Creates a new ElevatorToHeight. */
  public ElevatorToHeight(Elevator elevator, Double height) {
    m_elevator = elevator;
    heightSetpoint = height;

    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    heightController.setSetpoint(heightSetpoint);
    heightController.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var pwr = heightController.calculate(m_elevator.getElevatorHeightL());
    pwr = MathUtil.clamp(pwr, -0.4, 1.);

    m_elevator.setElevator(pwr);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    heightController.close();
    m_elevator.setElevator(kStallPower);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return heightController.atSetpoint();
  }
}
