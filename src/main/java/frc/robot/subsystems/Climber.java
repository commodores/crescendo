// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {

  private final CANSparkMax captainHookMotor;

  /** Creates a new Climber. */
  public Climber() {

    captainHookMotor = new CANSparkMax(Constants.ClimberConstants.captainHook, MotorType.kBrushless);
    captainHookMotor.restoreFactoryDefaults();
    captainHookMotor.setSmartCurrentLimit(30);
    captainHookMotor.setIdleMode(IdleMode.kBrake);

  }

   public void windUp(double speed){
    captainHookMotor.set(speed); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
