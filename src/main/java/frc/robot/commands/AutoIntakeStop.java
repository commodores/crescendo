// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoIntakeStop extends Command {
  /** Creates a new IntakeInn. */


  public AutoIntakeStop() {
    // Use addRequirements() here to declare subsystem dependencies.
   
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Intake.runIntakeSpeed(0);
    RobotContainer.m_Intake.runChooserSpeed(0);
    RobotContainer.m_Intake.runFeederSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}
