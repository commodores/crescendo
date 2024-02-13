// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {

  private final CANSparkMax intakeLeftMotor;
  private final CANSparkMax intakeRightMotor;
  private final CANSparkMax intakeChooserMotor;

  /** Creates a new Intake. */
  public Intake() {

    intakeLeftMotor = new CANSparkMax(Constants.IntakeConstants.intakeRight, MotorType.kBrushless);
    intakeLeftMotor.restoreFactoryDefaults();
    intakeLeftMotor.setSmartCurrentLimit(30);
    intakeLeftMotor.setIdleMode(IdleMode.kBrake);

    intakeRightMotor = new CANSparkMax(Constants.IntakeConstants.intakLeft, MotorType.kBrushless);
    intakeRightMotor.restoreFactoryDefaults();
    intakeRightMotor.setSmartCurrentLimit(30);
    intakeRightMotor.setIdleMode(IdleMode.kBrake);
    intakeRightMotor.follow(intakeLeftMotor,true);

    intakeChooserMotor = new CANSparkMax(Constants.IntakeConstants.intakeChooser, MotorType.kBrushless);
    intakeChooserMotor.restoreFactoryDefaults();
    intakeChooserMotor.setSmartCurrentLimit(30);
    intakeChooserMotor.setIdleMode(IdleMode.kBrake);
  }

  public void runIntakeSpeed(double speed){
    intakeLeftMotor.set(speed);
    intakeChooserMotor.set(speed);  
  }

  public void runChooserSpeed(double speed){
    intakeChooserMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 
} 