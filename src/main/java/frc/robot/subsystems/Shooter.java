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
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /**
   * Initializes the SparkMAX motor controller, assigns it to the CAN address
   * specified, and sets it to the NEO Brushless Motor.
   */
  private final CANSparkFlex shooterLeftMotor ;
  private final CANSparkFlex shooterRightMotor ;
  private final CANSparkFlex shooterAngleMotor ;
  private final CANSparkFlex shooterFeederMotor ;

  private double shooterSetpoint = 3750;
 
  
  /**
   * The built-in PID controller provided by the Spark MAX motor controller.
   */
  private final SparkPIDController shooterPIDLeft;
  /**
   * The target velocity of the NEO Brushless Motor.
   */
  
  /**
   * Scales the output of the SparkMAX PIDF controller.
   */
 
  /**
   * The maximum current the motor controller is allowed to feed to the motor, in
   * amps.
   */
  private final int currentLimit = 40;

  public Shooter() {

    shooterLeftMotor = new CANSparkFlex(Constants.ShooterConstants.shooterLeft, MotorType.kBrushless);
    shooterLeftMotor.restoreFactoryDefaults();
    shooterLeftMotor.setSmartCurrentLimit(40);

    shooterRightMotor = new CANSparkFlex(Constants.ShooterConstants.shooterRight, MotorType.kBrushless);
    shooterRightMotor.restoreFactoryDefaults();
    shooterRightMotor.setSmartCurrentLimit(40);
   

    shooterAngleMotor = new CANSparkFlex(Constants.ShooterConstants.shooterAngle, MotorType.kBrushless);
    shooterAngleMotor.restoreFactoryDefaults();
    shooterAngleMotor.setSmartCurrentLimit(40);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake);
    
    shooterFeederMotor = new CANSparkFlex(Constants.ShooterConstants.shooterFeeder, MotorType.kBrushless);
    shooterFeederMotor.restoreFactoryDefaults();
    shooterFeederMotor.setSmartCurrentLimit(40);
    shooterFeederMotor.setIdleMode(IdleMode.kBrake);
    
   
    shooterPIDLeft = shooterLeftMotor.getPIDController();
   // Applies the previously-declared values to the PIDF controller.
   shooterPIDLeft.setP(Constants.ShooterConstants.KP);
   shooterPIDLeft.setI(Constants.ShooterConstants.KI);
   shooterPIDLeft.setD(Constants.ShooterConstants.KD);
   shooterPIDLeft.setIZone(Constants.ShooterConstants.KIz);
   shooterPIDLeft.setFF(Constants.ShooterConstants.KFF);
   shooterPIDLeft.setOutputRange(Constants.ShooterConstants.KMinOutput, Constants.ShooterConstants.KMaxOutput);
   // Sets the shooter motor to coast so that subsequent shots don't have to rev up
   // from 0 speed.
   shooterLeftMotor.setIdleMode(IdleMode.kCoast);
   shooterLeftMotor.setIdleMode(IdleMode.kCoast);
   shooterLeftMotor.setSmartCurrentLimit(currentLimit);
   shooterLeftMotor.setSmartCurrentLimit(currentLimit);
   // Sets the left shooter motor to follow the right motor, and be inverted.
   shooterRightMotor.follow(shooterLeftMotor, true);
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



  @Override
  public void periodic() {
    SmartDashboard.putNumber("SetPoint", shooterSetpoint);
    SmartDashboard.putNumber("ProcessVariable", shooterLeftMotor.getEncoder().getVelocity());
  }
}