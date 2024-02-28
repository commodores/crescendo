// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntake extends Command {
  /** Creates a new IntakeInn. */

  private final Intake m_Intake;

  public AutoIntake(Intake intakeSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Intake = intakeSub;
    addRequirements(m_Intake);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.runIntakeSpeed(-1.0);
    m_Intake.runChooserSpeed(-1.0);
    m_Intake.runFeederSpeed(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.runIntakeSpeed(0);
    m_Intake.runChooserSpeed(0);
    m_Intake.runFeederSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
}
