// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Limelight;


public class AutoLEDTarget extends Command {

  private final Limelight m_Limelight;
  private final Blinkin m_Blinkin;
  double defult = 2000;

  /** Creates a new AutoShooterRPM. */
  public AutoLEDTarget(Limelight limelightSub, Blinkin blinkinSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Limelight = limelightSub;
    m_Blinkin = blinkinSub;
    addRequirements(m_Limelight);
    addRequirements(m_Blinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Blinkin.defult();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //Check for target
    if(m_Limelight.seesTarget()){
      //Check distance
      if(m_Limelight.getX()> -1 && m_Limelight.getX() < 1){
        m_Blinkin.green();
      } else {
        m_Blinkin.defult();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Blinkin.defult();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
