// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class AutoShooter extends Command {

  private final Shooter m_Shooter;
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
    powerLevel = RobotContainer.shooterPower.getDouble(1);
    
    m_Shooter.shoot(3500 * powerLevel);
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
