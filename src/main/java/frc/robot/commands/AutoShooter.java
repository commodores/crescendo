// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command {

  private final Shooter m_Shooter;
  //private final Limelight m_Limelight;
  double shooterSetPoint;
  double distance;
  double powerLevel;

  /** Creates a new AutoShooterRPM. */
  public AutoShooter(Shooter shooterSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = shooterSub;
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = RobotContainer.m_Limelight.getDistance();
    powerLevel = RobotContainer.shooterPower.getDouble(1);
    //KISS
    if(distance > 137.5 && distance < 200.0){
      shooterSetPoint = 3000.0;
    
    }else if(distance > 132.5 && distance < 137.5){
      shooterSetPoint = 3000.0;
    }else if(distance > 127.5 && distance < 132.5){
      shooterSetPoint = 3000.0;
    }else if(distance > 122.5 && distance < 127.5){
      shooterSetPoint = 3000.0;
    } else if(distance > 117.5 && distance < 122.5) {
      shooterSetPoint = 3000.0;
    }  else if(distance > 112.5 && distance < 117.5) {
      shooterSetPoint = 3000.0;
    } else if(distance > 107.5 && distance < 112.5) {
      shooterSetPoint = 3000.0;
    } else if(distance > 102.5 && distance < 107.5) {
      shooterSetPoint = 3000.0;
    } else if(distance > 97.5 && distance < 102.5) {
      shooterSetPoint = 3000.0;
    } else if(distance > 92.5 && distance < 97.5) {
      shooterSetPoint = 3000.0;
    } else if(distance > 87.5 && distance < 92.5) {
      shooterSetPoint = 3000.0;
    } else if(distance > 82.5 && distance < 87.5) {
      shooterSetPoint = 3000.0;
    } else if(distance > 77.5 && distance < 82.5) {
      shooterSetPoint = 3000.0;
    } else if(distance > 72.5 && distance < 77.5) {
      shooterSetPoint = 3000.0;
    } else if(distance > 67.5 && distance < 72.5) {
      shooterSetPoint = 2500.0;
    } else {
      shooterSetPoint = 2500.0;
    }

    m_Shooter.shoot(3000);
    //m_Shooter.shoot(shooterSetPoint*powerLevel);
    //System.out.println(m_Limelight.getDistance());
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
