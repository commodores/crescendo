// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AutoShooterRPM extends Command {

  private final Limelight m_Limelight;
  private final Shooter m_Shooter;
  double setpoint = 2000;

  /** Creates a new AutoShooterRPM. */
  public AutoShooterRPM(Limelight limelightSub, Shooter shooterSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = limelightSub;
    m_Shooter = shooterSub;
    addRequirements(m_Limelight);
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Shooter.shoot(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = m_Limelight.getDistance();
    //Check for target
    if(m_Limelight.seesTarget()){
      //Check distance
      if(distance>100){
        setpoint = 4000;
      } else if(distance > 80 && distance < 100){
        setpoint = 3000;
      }
    }
    //Set Speed
    m_Shooter.shoot(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
