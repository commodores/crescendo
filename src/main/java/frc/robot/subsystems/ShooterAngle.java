// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class ShooterAngle extends TrapezoidProfileSubsystem {

  private final CANSparkFlex shooterAngleMotor ;
  private final SparkPIDController shooterPIDAngle;
  private final RelativeEncoder m_relative_encoder;
 

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(1, 1));

    shooterAngleMotor = new CANSparkFlex(Constants.ShooterAngleConstants.shooterAngle, MotorType.kBrushless);

    shooterAngleMotor.restoreFactoryDefaults();
    shooterAngleMotor.setInverted(false);
    shooterAngleMotor.setSmartCurrentLimit(100);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake);

    shooterPIDAngle = shooterAngleMotor.getPIDController();
    shooterPIDAngle.setP(Constants.ShooterAngleConstants.KP);
    shooterPIDAngle.setI(Constants.ShooterAngleConstants.KI);
    shooterPIDAngle.setD(Constants.ShooterAngleConstants.KD);
    shooterPIDAngle.setFF(Constants.ShooterAngleConstants.KFF);

    shooterPIDAngle.setOutputRange(Constants.ShooterAngleConstants.KMinOutput, Constants.ShooterAngleConstants.KMaxOutput);

    m_relative_encoder = shooterAngleMotor.getEncoder();
    m_relative_encoder.setPositionConversionFactor((2 * Math.PI) / Constants.ShooterAngleConstants.kGearRatio); //Converted to Radians
    m_relative_encoder.setPosition(.61);

    shooterPIDAngle.setFeedbackDevice(m_relative_encoder);
    
  }

    @Override
  public void periodic() {
    double relativeEncoderValue = m_relative_encoder.getPosition();    
    
    // Display current values on the SmartDashboard
    //SmartDashboard.putNumber("arm Output", shoulderMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter Angle Degrees", Units.radiansToDegrees(relativeEncoderValue));
    SmartDashboard.putNumber("Shooter Angles Radians", relativeEncoderValue);
    
    // Execute the super class periodic method
    super.periodic();
  }


  @Override
  protected void useState(TrapezoidProfile.State setPoint) {
    shooterPIDAngle.setReference(setPoint.position, CANSparkFlex.ControlType.kPosition, 0);
  }

  public Command setArmGoalCommand(double goal) {
   return Commands.runOnce(() -> setGoal(goal), this);
  }

  public double getEncoder(){
    return m_relative_encoder.getPosition();
  }


}
