// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

public class AutoShooter extends Command {

  private final Limelight m_Limelight;
  private final Shooter m_Shooter;
  private final ShooterAngle m_ShooterAngle;
  double shooterSetPoint = 2000;
  double angleSetPoint = .61;
  double distance = 0;

  /** Creates a new AutoShooterRPM. */
  public AutoShooter(Limelight limelightSub, Shooter shooterSub, ShooterAngle angleSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = limelightSub;
    m_Shooter = shooterSub;
    m_ShooterAngle = angleSub;
    addRequirements(m_Limelight);
    addRequirements(m_Shooter);
    addRequirements(m_ShooterAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Shooter.shoot(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    //Check for target
    if(m_Limelight.seesTarget()){
      //Check distance
      distance = m_Limelight.getDistance();
      
      //KISS
      if(distance > 250){
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
