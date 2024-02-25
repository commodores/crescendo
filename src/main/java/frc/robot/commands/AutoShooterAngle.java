// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngle;

public class AutoShooterAngle extends Command {

  private final ShooterAngle m_ShooterAngle;
  double angleSetPoint = .61;

  /** Creates a new AutoShooterRPM. */
  public AutoShooterAngle(ShooterAngle angleSub, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterAngle = angleSub;
    addRequirements(m_ShooterAngle);
      
    //KISS
    if(distance > 250 && distance < 400){
      angleSetPoint = 0.0;
    }else if(distance > 225 && distance < 250){
      angleSetPoint = 0.04;
    }else if(distance > 200 && distance < 225){
      angleSetPoint = 0.1;
    }else if(distance > 175 && distance < 200){
      angleSetPoint = 0.17;
    } else if(distance > 150 && distance < 175) {
      angleSetPoint = 0.24;
    }  else if(distance > 125 && distance < 150) {
      angleSetPoint = 0.31;
    } else if(distance > 100 && distance < 125) {
      angleSetPoint = 0.37;
    } else if(distance > 75 && distance < 100) {
      angleSetPoint = 0.61;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterAngle.setGoal(angleSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterAngle.setGoal(angleSetPoint);
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
