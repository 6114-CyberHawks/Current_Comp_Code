// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class AutonomousC extends CommandBase {
  Intake intake;
  Timer timer;
  /** Creates a new AutonomousC. */
  public AutonomousC(Intake IT) {
    intake = IT;
    addRequirements(intake);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    Arm.pid.setSetpoint(0.28);
    intake.idle(.15);
    if(timer.get() > 1 & timer.get() < 1.5) {
      intake.dropObject(-.3);
    } if (timer.get() > 1.5) {
      intake.stop();
    } if (timer.get() > 1.6) {
      Arm.pid.setSetpoint(0);
    }
    if (timer.get() > 2.7 & timer.get() < 3.7) {
      RobotContainer.m_robotDrive.drive(-.3, 0, 0, true, true);
    }
    else {
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
