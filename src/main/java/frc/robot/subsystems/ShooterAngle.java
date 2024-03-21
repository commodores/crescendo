// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterAngle extends SubsystemBase {

  private final CANSparkFlex shooterAngleMotor;
  private final RelativeEncoder m_angle_encoder;
  private final SparkPIDController shooterPIDAngle;
  private final DigitalInput upLimitSwitch, downLimitSwitch;

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {


//Angle Motor
    shooterAngleMotor = new CANSparkFlex(Constants.ShooterConstants.shooterAngle, MotorType.kBrushless);

    shooterAngleMotor.restoreFactoryDefaults();
    shooterAngleMotor.setInverted(false);
    shooterAngleMotor.setSmartCurrentLimit(40);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake);

    shooterPIDAngle = shooterAngleMotor.getPIDController();
    shooterPIDAngle.setP(Constants.ShooterConstants.ANGLEKP);
    shooterPIDAngle.setI(Constants.ShooterConstants.ANGLEKI);
    shooterPIDAngle.setD(Constants.ShooterConstants.ANGLEKD);
    shooterPIDAngle.setFF(Constants.ShooterConstants.ANGLEKFF);

    shooterPIDAngle.setOutputRange(Constants.ShooterConstants.ANGLEKMinOutput, Constants.ShooterConstants.ANGLEKMaxOutput);

    m_angle_encoder = shooterAngleMotor.getEncoder();
    m_angle_encoder.setPositionConversionFactor((2 * Math.PI) / Constants.ShooterConstants.kAngleGearRatio); //Converted to Radians
    
    upLimitSwitch = new DigitalInput(0);
    downLimitSwitch = new DigitalInput(1);

    shooterPIDAngle.setFeedbackDevice(m_angle_encoder);
    resetAngleUp(); 

  }

  public void setShooterAngle(double degrees){
    shooterPIDAngle.setReference(Math.toRadians(degrees), CANSparkFlex.ControlType.kPosition, 0);
  }

  public void resetAngleUp(){
    m_angle_encoder.setPosition(Math.toRadians(60));
  }

  public void resetAngleDown(){
    m_angle_encoder.setPosition(Math.toRadians(28.5));
  }

  public boolean getUpLimitSwitch(){
    return !upLimitSwitch.get();
  }

  public boolean getDownLimitSwitch(){
    return !downLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Angle Degrees", Units.radiansToDegrees(m_angle_encoder.getPosition()));
    SmartDashboard.putNumber("Shooter Angles Radians", m_angle_encoder.getPosition());

    SmartDashboard.putBoolean("Up Limit", getUpLimitSwitch());
    SmartDashboard.putBoolean("Down Limit", getDownLimitSwitch());

    if(getUpLimitSwitch()){
      resetAngleUp();
    }

    if(getDownLimitSwitch()){
      resetAngleDown();
    }

  }
}
