// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoShooterRPM;
import frc.robot.commands.ShooterIntake;
import frc.robot.commands.StopAll;
import frc.robot.commands.StopAllShooter;
import frc.robot.commands.TrampIntake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.TrampElevator;
import frc.robot.subsystems.Trampinator;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // Driver
  private final CommandXboxController joystick2 = new CommandXboxController(1); // Secondary

  //Create Subsystems
  public final CommandSwerveDrivetrain m_Drivetrain = TunerConstants.DriveTrain; // My drivetrain  
  public static final Intake m_Intake = new Intake();
  public static final Trampinator m_Trampinator = new Trampinator();
  public static final TrampElevator m_TrampElevator = new TrampElevator();
  public static final Shooter m_Shooter = new Shooter();
  public static final Climber m_Climber = new Climber();
  public static final ShooterAngle m_ShooterAngle = new ShooterAngle();
  public static final Limelight m_Limelight = new Limelight();

  //Drive Swerve
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    //Set Default Commands
    m_Drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        m_Drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    //m_Shooter.setDefaultCommand(new AutoShooterRPM(m_Limelight, m_Shooter));
    
    //Button Bindings
    joystick.a().whileTrue(m_Drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(m_Drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.start().onTrue(m_Drivetrain.runOnce(() -> m_Drivetrain.seedFieldRelative()));

    /* Intake Commands */
    joystick2.a().onTrue(new ShooterIntake(m_Intake, m_Shooter).withTimeout(5));
    joystick2.a().onFalse(new StopAllShooter(m_Intake, m_Shooter).withTimeout(5));

    joystick2.b().onTrue(new TrampIntake(m_Intake, m_Trampinator).withTimeout(5));
    joystick2.b().onFalse(new StopAll(m_Intake, m_Trampinator).withTimeout(5));
    

    /*Trampinator Commands */
    joystick2.x().whileTrue(new InstantCommand(() -> m_Trampinator.runShooterSpeed(1)));
    joystick2.y().whileTrue(new InstantCommand(() -> m_Trampinator.runShooterSpeed(-1)));

    joystick2.x().onFalse(new InstantCommand(() -> m_Trampinator.runShooterSpeed(0)));
    joystick2.y().onFalse(new InstantCommand(() -> m_Trampinator.runShooterSpeed(0)));

    /*Tramp Elevator Commands */
    joystick2.rightBumper().onTrue(m_TrampElevator.setElevatorGoalCommand(0.35));
    joystick2.leftBumper().onTrue(m_TrampElevator.setElevatorGoalCommand(0.0));

    /*Shooter Commands */
    joystick.x().whileTrue(new InstantCommand(() ->m_Shooter.shoot(3600)));//5000
    joystick.x().onFalse(new InstantCommand(() ->m_Shooter.stopShooter()));

    joystick2.start().onTrue(m_ShooterAngle.setArmGoalCommand(.37));
    joystick2.back().onTrue(m_ShooterAngle.setArmGoalCommand(0));

    //joystick2.start().whileTrue(new InstantCommand(() -> m_ShooterAngle.runAngleSpeed(.5)));
    //joystick2.start().onFalse(new InstantCommand(() -> m_ShooterAngle.runAngleSpeed(0)));
    
    //joystick2.back().whileTrue(new InstantCommand(() -> m_ShooterAngle.runAngleSpeed(-.5)));
    //joystick2.back().onFalse(new InstantCommand(() -> m_ShooterAngle.runAngleSpeed(0)));


    /*Climber Commands */
    joystick.rightBumper().whileTrue(new InstantCommand(() -> m_Climber.windUp(0.5)));
    joystick.rightBumper().onFalse(new InstantCommand(() -> m_Climber.windUp(0)));
    
    joystick.leftBumper().whileTrue(new InstantCommand(() -> m_Climber.windUp(-0.5)));
    joystick.leftBumper().onFalse(new InstantCommand(() -> m_Climber.windUp(0)));

    joystick2.povDown().whileTrue(new InstantCommand(() -> m_Climber.climbUp(-0.5)));
    joystick2.povDown().onFalse(new InstantCommand(() -> m_Climber.climbUp(0)));

    joystick2.povUp().whileTrue(new InstantCommand(() -> m_Climber.climbUp(0.5)));
    joystick2.povUp().onFalse(new InstantCommand(() -> m_Climber.climbUp(0)));

    if (Utils.isSimulation()) {
      m_Drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_Drivetrain.registerTelemetry(logger::telemeterize); 
  }

  public RobotContainer() {
    configureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
