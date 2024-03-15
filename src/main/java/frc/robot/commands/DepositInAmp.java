// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import static frc.robot.Constants.Shooter.kAmpAngle;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.PathfindingPoints.Red;
// import frc.robot.Constants.PathfindingPoints.Blue;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DepositInAmp extends SequentialCommandGroup {
  // private Pose2d ampPoint = Red.Amp;
  /** Creates a new DepositInAmp. */
  public DepositInAmp(Drivetrain drivetrain, Intake intake, Loader loader, Turret turret, Shooter shooter, Elevator elevator, Trigger shootButton) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      // new InstantCommand(() -> {
      //   if (DriverStation.getAlliance().isPresent()) {
      //     if (DriverStation.getAlliance().get() == Alliance.Red) {
      //       ampPoint = Red.Amp;
      //     } else {
      //       ampPoint = Blue.Amp;
      //     }
      //   }
      // }),
      new ParallelDeadlineGroup(
        new ElevatorToHeight(elevator, 6.5),
        new SetTurretAngle(turret, 50.)
        ).withTimeout(1.),
      new ParallelDeadlineGroup(
        new ShootSpeedAngleWithControl(shooter, turret, loader, 2000., kAmpAngle, shootButton, true),
        new ElevatorToHeight(elevator, 6.5)),
      new ElevatorToHeight(elevator, 0.1)
        // new DriveToPoint(drivetrain, ampPoint).withTimeout(1.5),
    );
  }
}
