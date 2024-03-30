// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterAngle;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoShooterAngle extends Command {

  private final ShooterAngle m_ShooterAngle;
  double angle;
  double setPoint;
  double distanceToGoal;
 
  /** Creates a new AutoShooterAngle. */
  public AutoShooterAngle(ShooterAngle angleSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterAngle = angleSub;
    addRequirements(m_ShooterAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = RobotContainer.m_Limelight.getShooterAngle();

    distanceToGoal = RobotContainer.m_Limelight.getDistance();

    if(distanceToGoal < 300){
      if(angle < 28.5){
        setPoint = 28.5;
      } else if(angle > 60) {
        setPoint = 60;
      }else{
        setPoint = angle;
      }
    } else {
      setPoint = 60;
    }

    m_ShooterAngle.setShooterAngle(setPoint);

    //System.out.println(setPoint);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
