// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends ParallelCommandGroup {
  private final Limelight m_Limelight;
  /** Creates a new AutoShoot. */
  public AutoShoot(Limelight limelightSub) {
    m_Limelight = limelightSub;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShooter(RobotContainer.m_Shooter, m_Limelight.getDistance()),
      new AutoShooterAngle(RobotContainer.m_ShooterAngle, m_Limelight.getDistance())
    );
  }
}
