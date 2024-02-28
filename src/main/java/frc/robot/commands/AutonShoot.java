// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class AutonShoot extends Command {

  private final Shooter m_Shooter;
  //private final Limelight m_Limelight;
  double shooterSetPoint;
  double angleSetPoint;
  double distance;

  /** Creates a new AutoShooterRPM. */
  public AutonShoot(Shooter shooterSub) {
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
    //KISS
    if(distance > 150.0 && distance < 200.0){
      shooterSetPoint = 3000.0;
      angleSetPoint = 0.04;
    }else if(distance > 145.0 && distance < 150.0){
      shooterSetPoint = 3000.0;
      angleSetPoint = 0.04;
    }else if(distance > 140.0 && distance < 145.0){
      shooterSetPoint = 3000.0;
      angleSetPoint = 0.04;
    }else if(distance > 135.0 && distance < 140.0){
      shooterSetPoint = 3000.0;
      angleSetPoint = 0.04;
    } else if(distance > 130.0 && distance < 135.0) {
      shooterSetPoint = 3000.0;
      angleSetPoint = 0.04;
    }  else if(distance > 125.0 && distance < 130.0) {
      shooterSetPoint = 3000.0;
      angleSetPoint = 0.04;
    } else if(distance > 120.0 && distance < 125.0) {
      shooterSetPoint = 3000.0;
      angleSetPoint = 0.05;
    } else if(distance > 115.0 && distance < 120.0) {
      shooterSetPoint = 3000.0;
      angleSetPoint = 0.067;
    } else if(distance > 110.0 && distance < 115.0) {
      shooterSetPoint = 2700.0;
      angleSetPoint = 0.0705;
    } else if(distance > 105.0 && distance < 110.0) {
      shooterSetPoint = 2700.0;
      angleSetPoint = 0.0775;
    } else if(distance > 100.0 && distance < 105) {
      shooterSetPoint = 2700.0;
      angleSetPoint = 0.1;
    } else if(distance > 95.0 && distance < 100.0) {
      shooterSetPoint = 2700.0;
      angleSetPoint = 0.15;
    } else if(distance > 82.5 && distance < 95.0) {
      shooterSetPoint = 2700.0;
      angleSetPoint = 0.2;
    } else if(distance > 76.0 && distance < 82.5) {
      shooterSetPoint = 2700.0;
      angleSetPoint = 0.3;
    } else if(distance > 70.0 && distance < 76) {
      shooterSetPoint = 2700.0;
      angleSetPoint = 0.35;
    } else {
      shooterSetPoint = 2500.0;
      angleSetPoint = 0.525;
    }

    m_Shooter.shoot(shooterSetPoint);
    m_Shooter.setShooterAngle(angleSetPoint);
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
