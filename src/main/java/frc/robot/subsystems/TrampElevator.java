// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
 

  private final ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.TrampinatorConstants.kSVolts, Constants.TrampinatorConstants.kGVolts,
          Constants.TrampinatorConstants.kVVoltSecondPerRad, Constants.TrampinatorConstants.kAVoltSecondSquaredPerRad);

  /** Creates a new ArmShoulder. */
  public TrampElevator() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(0, 0),
        // The initial position of the mechanism
        Math.PI);

    shoulderMotor = new CANSparkMax(Constants.TrampinatorConstants.elevator, MotorType.kBrushless);
    
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setInverted(false);
    shoulderMotor.setSmartCurrentLimit(1);
    shoulderMotor.setIdleMode(IdleMode.kBrake);

    m_PIDController = shoulderMotor.getPIDController();
    m_PIDController.setP(Constants.TrampinatorConstants.KP);
    m_PIDController.setI(Constants.TrampinatorConstants.KI);
    m_PIDController.setD(Constants.TrampinatorConstants.KD);
    m_PIDController.setFF(Constants.TrampinatorConstants.KFF);

    m_relative_encoder = shoulderMotor.getEncoder();
    m_relative_encoder.setPositionConversionFactor((2 * Math.PI) / Constants.TrampinatorConstants.kGearRatio); //Converted to Radians

    m_PIDController.setFeedbackDevice(m_relative_encoder);
    

  }

  @Override
  public void periodic() {
    double relativeEncoderValue = m_relative_encoder.getPosition();
    //syncEncoder();
    
    // Display current values on the SmartDashboard
    //SmartDashboard.putNumber("arm Output", shoulderMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shoulder Relative Encoder Degrees", Units.radiansToDegrees(relativeEncoderValue));
    SmartDashboard.putNumber("Shoulder Relative Encoder Radians", relativeEncoderValue);
    SmartDashboard.putNumber("Shoulder WAWA", getEncoder());

    // Execute the super class periodic method
    super.periodic();
  }

  @Override
  protected void useState(TrapezoidProfile.State setPoint) {
    // Calculate the feedforward fromteh setPoint
    double feedforward = m_feedforward.calculate(setPoint.position, setPoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    // The ArmFeedForward computes in radians. We need to convert back to degrees.
    // Remember that the encoder was already set to account for the gear ratios.
    
    m_PIDController.setReference(setPoint.position, CANSparkMax.ControlType.kPosition, 0);
    
    //SmartDashboard.putNumber("Shoulder Feedforward", feedforward);
    //SmartDashboard.putNumber("Shoulder SetPoint", Units.metersToInches(setPoint.position));
    //SmartDashboard.putNumber("Shoulder Velocity", Units.metersToInches(setPoint.velocity));
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }

  public double getEncoder(){
    return m_relative_encoder.getPosition();
  }
}