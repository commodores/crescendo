// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.CANdleSub;

public class AutoLEDTarget extends Command {

  private final CANdleSub m_CANdleSub;
  double error;
  Optional<Alliance> ally;

  /** Creates a new AutoShooterRPM. */
  public AutoLEDTarget(CANdleSub CANdleSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_CANdleSub = CANdleSub;
    addRequirements(m_CANdleSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            m_CANdleSub.setColor(255, 0, 0);
        }
        if (ally.get() == Alliance.Blue) {
            m_CANdleSub.setColor(0, 0, 255);
        }
    }
    else {
        m_CANdleSub.setColor(200, 200, 0);
    }   
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = RobotContainer.m_Limelight.getX();
    //Check error
    if(RobotContainer.m_Limelight.seesTarget()){
      if(error > -3 && error < 3){
        m_CANdleSub.setColor(0, 255, 0);
      } else {
        if (ally.get() == Alliance.Red) {
            m_CANdleSub.setColor(255, 0, 0);
        }
        if (ally.get() == Alliance.Blue) {
            m_CANdleSub.setColor(0, 0, 255);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CANdleSub.setColor(200, 200, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
