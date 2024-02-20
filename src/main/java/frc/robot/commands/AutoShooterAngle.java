// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterAngle;

public class AutoShooterAngle extends Command {

  private final Limelight m_Limelight;
  private final ShooterAngle m_ShooterAngle;
  double setpoint = .61;

  /** Creates a new AutoShooterRPM. */
  public AutoShooterAngle(Limelight limelightSub, ShooterAngle angleSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = limelightSub;
    m_ShooterAngle = angleSub;
    addRequirements(m_Limelight);
    addRequirements(m_ShooterAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //  m_ShooterAngle.setArmGoalCommand(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Check for target
   
      //Check distance
      if(m_Limelight.getDistance()>100 && m_Limelight.getDistance() < 200){
        setpoint = .37;
      } else if(m_Limelight.getDistance() > 200 && m_Limelight.getDistance() < 220){
        setpoint = .1;
      } else if(m_Limelight.getDistance() > 220){
        setpoint = 0;
      }else{
        setpoint = .61;
      }
    
    //Set Speed
    m_ShooterAngle.setArmGoalCommand(setpoint);
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
