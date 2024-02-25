// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command {

  private final Shooter m_Shooter;
  double shooterSetPoint = 2000;

  /** Creates a new AutoShooterRPM. */
  public AutoShooter(Shooter shooterSub, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = shooterSub;
    addRequirements(m_Shooter);
      
    //KISS
    if(distance > 250 && distance < 400){
      shooterSetPoint = 2750;
    }else if(distance > 225 && distance < 400){
      shooterSetPoint = 2500;
    }else if(distance > 200 && distance < 225){
      shooterSetPoint = 2250;
    }else if(distance > 175 && distance < 200){
      shooterSetPoint = 2150;
    } else if(distance > 150 && distance < 175) {
      shooterSetPoint = 2000;
    }  else if(distance > 125 && distance < 150) {
      shooterSetPoint = 2000;
    } else if(distance > 100 && distance < 125) {
      shooterSetPoint = 2000;
    } else if(distance > 75 && distance < 100) {
      shooterSetPoint = 2000;
    }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Shooter.shoot(shooterSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    //Check for target
    if(m_Limelight.seesTarget()){
      //Check distance
      distance = m_Limelight.getDistance();
      
      //KISS
      if(distance > 250 && distance < 350){
        shooterSetPoint = 5500;
        angleSetPoint = 0.0;
      }else if(distance > 225){
        shooterSetPoint = 5000;
        angleSetPoint = 0.04;
      }else if(distance > 200){
        shooterSetPoint = 5000;
        angleSetPoint = 0.1;
      }else if(distance > 175){
        shooterSetPoint = 5000;
        angleSetPoint = 0.17;
      } else if(distance > 150) {
        shooterSetPoint = 5000;
        angleSetPoint = 0.24;
      }  else if(distance > 125) {
        shooterSetPoint = 5000;
        angleSetPoint = 0.31;
      } else if(distance > 100) {
        shooterSetPoint = 4000;
        angleSetPoint = 0.37;
      } else if(distance > 75) {
        shooterSetPoint = 4000;
        angleSetPoint = 0.61;
      }
    
      //Set Speed
      m_Shooter.shoot(shooterSetPoint);
      //Set Angle
      m_ShooterAngle.setGoal(angleSetPoint);
    }

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
