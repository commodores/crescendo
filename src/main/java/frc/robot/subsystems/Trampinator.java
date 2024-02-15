// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Trampinator extends SubsystemBase {

  private final CANSparkFlex shooterMotor;
  /** Creates a new Trampinator. */
  public Trampinator() {

     shooterMotor = new CANSparkFlex(Constants.TrampinatorConstants.shooter, MotorType.kBrushless);
     shooterMotor.restoreFactoryDefaults();
     shooterMotor.setSmartCurrentLimit(100);
     shooterMotor.setIdleMode(IdleMode.kBrake);
  }


  public void runShooterSpeed(double speed){
    shooterMotor.set(speed); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
