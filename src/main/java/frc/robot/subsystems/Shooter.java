package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /**
   * Initializes the SparkMAX motor controller, assigns it to the CAN address
   * specified, and sets it to the NEO Brushless Motor.
   */
  private final CANSparkFlex shooterLeftMotor;
  private final CANSparkFlex shooterRightMotor;
  
  
  private final RelativeEncoder m_left_encoder;
  private final RelativeEncoder m_right_encoder;
 
 
  private final SparkPIDController shooterPIDLeft;
  private final SparkPIDController shooterPIDRight;
  


  public Shooter() {

    //Shooter Motors
    shooterLeftMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLeft, MotorType.kBrushless);
    shooterLeftMotor.restoreFactoryDefaults();
    shooterLeftMotor.setSmartCurrentLimit(80);
    shooterLeftMotor.setIdleMode(IdleMode.kCoast);
    shooterLeftMotor.setInverted(false);
    shooterPIDLeft = shooterLeftMotor.getPIDController();
    // Applies the previously-declared values to the PIDF controller.
    shooterPIDLeft.setP(Constants.ShooterConstants.KP);
    shooterPIDLeft.setI(Constants.ShooterConstants.KI);
    shooterPIDLeft.setD(Constants.ShooterConstants.KD);
    shooterPIDLeft.setIZone(Constants.ShooterConstants.KIz);
    shooterPIDLeft.setFF(Constants.ShooterConstants.KFF);
    shooterPIDLeft.setOutputRange(Constants.ShooterConstants.KMinOutput, Constants.ShooterConstants.KMaxOutput);

    shooterRightMotor = new CANSparkFlex(Constants.ShooterConstants.shooterRight, MotorType.kBrushless);
    shooterRightMotor.restoreFactoryDefaults();
    shooterRightMotor.setSmartCurrentLimit(80);
    shooterRightMotor.setIdleMode(IdleMode.kCoast);
    shooterRightMotor.setInverted(true);
    shooterPIDRight = shooterRightMotor.getPIDController();
    // Applies the previously-declared values to the PIDF controller.
    shooterPIDRight.setP(Constants.ShooterConstants.KP);
    shooterPIDRight.setI(Constants.ShooterConstants.KI);
    shooterPIDRight.setD(Constants.ShooterConstants.KD);
    shooterPIDRight.setIZone(Constants.ShooterConstants.KIz);
    shooterPIDRight.setFF(Constants.ShooterConstants.KFF);
    shooterPIDRight.setOutputRange(Constants.ShooterConstants.KMinOutput, Constants.ShooterConstants.KMaxOutput); 

    m_left_encoder = shooterLeftMotor.getEncoder();
    m_right_encoder = shooterRightMotor.getEncoder();
    
  }

  /**
   * Passes a preset velocity to the SparkMAX PIDF controller and lets it manage
   * the NEO's velocity. Intended to be called when a button is pressed.
   */
  public void shoot(double setPoint) {
    shooterPIDLeft.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    shooterPIDRight.setReference(setPoint-1000, CANSparkMax.ControlType.kVelocity);
  }

  public void shootClose(double setPoint) {
    shooterPIDLeft.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    shooterPIDRight.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * Stops the shooter motor. Note: the NEO is set to Coast. Intended to be called
   * when a button is released.
   */
  public void stopShooter() {
    shooterLeftMotor.stopMotor();
    shooterRightMotor.stopMotor();
  }


  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Shooter Velocity", m_left_encoder.getVelocity());
    SmartDashboard.putNumber("Right Shooter Velocity", m_right_encoder.getVelocity()); 
  }
}