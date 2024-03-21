// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;


public class AutoAim extends Command {
  /** Creates a new AutoAim. */
  
  
  public AutoAim() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_Drivetrain.applyRequest(() -> 
      RobotContainer.drive.withVelocityX(0)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(RobotContainer.m_Limelight.LimelightAim()) // Drive counterclockwise with negative X (left)
        );
  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_Limelight.isAimed();
  }
}
