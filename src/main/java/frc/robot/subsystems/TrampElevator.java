// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class TrampElevator extends TrapezoidProfileSubsystem {

  // Define the motor and encoders
  private final CANSparkMax shoulderMotor;
  private final SparkPIDController m_PIDController;
  private final RelativeEncoder m_relative_encoder;

  /** Creates a new ArmShoulder. */
  public TrampElevator() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(2.54, 2.54));

    shoulderMotor = new CANSparkMax(Constants.TrampinatorConstants.elevator, MotorType.kBrushless);
    
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setInverted(true);
    shoulderMotor.setSmartCurrentLimit(35);
    shoulderMotor.setIdleMode(IdleMode.kBrake);

    m_PIDController = shoulderMotor.getPIDController();
    m_PIDController.setP(Constants.TrampinatorConstants.KP);
    m_PIDController.setI(Constants.TrampinatorConstants.KI);
    m_PIDController.setD(Constants.TrampinatorConstants.KD);
    m_PIDController.setFF(Constants.TrampinatorConstants.KFF);

    m_relative_encoder = shoulderMotor.getEncoder();
   //m_relative_encoder.setInverted(true);
    m_relative_encoder.setPositionConversionFactor(Constants.TrampinatorConstants.kMeterPerRevolution); 

    m_PIDController.setFeedbackDevice(m_relative_encoder);    

  }

  @Override
  public void periodic() {
    // Display current values on the SmartDashboard
    
    SmartDashboard.putNumber("Elevator Raw", getEncoder());

    // Execute the super class periodic method
    super.periodic();
  }

  @Override
  protected void useState(TrapezoidProfile.State setPoint) {
    m_PIDController.setReference(setPoint.position, CANSparkMax.ControlType.kPosition, 0);
  }

  public Command setElevatorGoalCommand(double goal) {
    return Commands.runOnce(() -> setGoal(goal), this);
  }

  public double getEncoder(){
    return m_relative_encoder.getPosition();
  }
}