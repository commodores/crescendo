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
  private final CANSparkFlex shooterLeftMotor ;
  private final CANSparkFlex shooterRightMotor ;
  private final CANSparkFlex shooterFeederMotor ;

  private double shooterSetpoint = 3750;
 private final RelativeEncoder m_relative_encoder;
 
  private final SparkPIDController shooterPIDLeft;

  public Shooter() {

    shooterLeftMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLeft, MotorType.kBrushless);
    shooterLeftMotor.restoreFactoryDefaults();
    shooterLeftMotor.setSmartCurrentLimit(40);
    shooterLeftMotor.setIdleMode(IdleMode.kCoast);
    shooterLeftMotor.setInverted(true);
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
    shooterRightMotor.setSmartCurrentLimit(40);
    shooterLeftMotor.setIdleMode(IdleMode.kCoast);
    shooterRightMotor.follow(shooterLeftMotor, false);   

    m_relative_encoder = shooterLeftMotor.getEncoder();
    
    shooterFeederMotor = new CANSparkFlex(Constants.ShooterConstants.shooterFeeder, MotorType.kBrushless);
    shooterFeederMotor.restoreFactoryDefaults();
    shooterFeederMotor.setSmartCurrentLimit(100);
    shooterFeederMotor.setIdleMode(IdleMode.kBrake);
   
  }

  /**
   * Passes a preset velocity to the SparkMAX PIDF controller and lets it manage
   * the NEO's velocity. Intended to be called when a button is pressed.
   */
  public void shoot(double setPoint) {
    shooterPIDLeft.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * Stops the shooter motor. Note: the NEO is set to Coast. Intended to be called
   * when a button is released.
   */
  public void stopShooter() {
    shooterLeftMotor.stopMotor();
  }

  public void setSetpoint(double newPoint) {
    shooterSetpoint = newPoint;
  }

  public void runFeederSpeed(double speed){
    shooterFeederMotor.set(speed);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("SetPoint", shooterSetpoint);
    SmartDashboard.putNumber("Velocity", shooterLeftMotor.getEncoder().getVelocity());
  }
}