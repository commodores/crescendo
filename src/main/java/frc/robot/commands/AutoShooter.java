// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;

public class AutoShooter extends Command {

  private final Limelight m_Limelight;
  private final Shooter m_Shooter;
  private final ShooterAngle m_ShooterAngle;
  double shootersetpoint = 4000;
  double anglesetpoint = .61;

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
    //if(m_Limelight.seesTarget()){
      //Check distance
    if(m_Limelight.getDistance()>150){
      shootersetpoint = 5000;
    } else {
        shootersetpoint = 4000;
    }
    
    //Set Speed
   m_Shooter.shoot(shootersetpoint);

    //Shooter Angle
    if(m_Limelight.getDistance()>100 && m_Limelight.getDistance() < 200){
      anglesetpoint = .37;
    } else if(m_Limelight.getDistance() > 200 && m_Limelight.getDistance() < 220){
      anglesetpoint = .1;
    } else if(m_Limelight.getDistance() > 220){
      anglesetpoint = 0;
    }else{
      anglesetpoint = .61;
    }
  
  //Set Angle
  m_ShooterAngle.setGoal(anglesetpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_Shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
