// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command {

  private final Shooter m_Shooter;
  private final Limelight m_Limelight;
  double shooterSetPoint;
  double angleSetPoint;
  double distance;

  /** Creates a new AutoShooterRPM. */
  public AutoShooter(Shooter shooterSub, Limelight limelightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Shooter = shooterSub;
    m_Limelight = limelightSub;
    addRequirements(m_Shooter);
    addRequirements(m_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = m_Limelight.getDistance();
    //KISS
    if(distance > 150.0 && distance < 200.0){
      shooterSetPoint = 2665.0;
      angleSetPoint = 0.0;
    }else if(distance > 145.0 && distance < 150.0){
      shooterSetPoint = 2650.0;
      angleSetPoint = 0.0;
    }else if(distance > 140.0 && distance < 145.0){
      shooterSetPoint = 2650.0;
      angleSetPoint = 0.02;
    }else if(distance > 135.0 && distance < 140.0){
      shooterSetPoint = 2650.0;
      angleSetPoint = 0.04;
    } else if(distance > 130.0 && distance < 135.0) {
      shooterSetPoint = 2650.0;
      angleSetPoint = 0.06;
    }  else if(distance > 125.0 && distance < 130.0) {
      shooterSetPoint = 2650.0;
      angleSetPoint = 0.08;
    } else if(distance > 120.0 && distance < 125.0) {
      shooterSetPoint = 2650.0;
      angleSetPoint = 0.1;
    } else if(distance > 115.0 && distance < 120.0) {
      shooterSetPoint = 2650.0;
      angleSetPoint = 0.12;
    } else if(distance > 110.0 && distance < 115.0) {
      shooterSetPoint = 2600.0;
      angleSetPoint = 0.14;
    } else if(distance > 105.0 && distance < 110.0) {
      shooterSetPoint = 2600.0;
      angleSetPoint = 0.16;
    } else if(distance > 95.0 && distance < 105.0) {
      shooterSetPoint = 2500.0;
      angleSetPoint = 0.22;
    } else if(distance > 82.5 && distance < 95.0) {
      shooterSetPoint = 2450.0;
      angleSetPoint = 0.28;
    } else if(distance > 70.0 && distance < 82.5) {
      shooterSetPoint = 2300.0;
      angleSetPoint = 0.57;
    } else {
      shooterSetPoint = 2300.0;
      angleSetPoint = 0.61;
    }

    m_Shooter.shoot(shooterSetPoint);
    m_Shooter.setShooterAngle(angleSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shooter.shoot(shooterSetPoint);
    m_Shooter.setShooterAngle(angleSetPoint);
    System.out.println(m_Limelight.getDistance());
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
