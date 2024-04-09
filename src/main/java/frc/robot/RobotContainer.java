// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoIntakeStop;
import frc.robot.commands.AutoLEDTarget;
import frc.robot.commands.AutoShooter;
import frc.robot.commands.AutoShooterAngle;
import frc.robot.commands.AutoSlowShoot;
import frc.robot.commands.AutoStopShooter;
import frc.robot.commands.GotIt;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ShooterIntake;
import frc.robot.commands.StopAll;
import frc.robot.commands.StopAllShooter;
import frc.robot.commands.AmpIntake;
import frc.robot.commands.AmpIntakeManual;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.CANdleSub;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightRear;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.AmpElevator;
import frc.robot.subsystems.Ampinator;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  public static final  double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // Driver
  private final CommandXboxController joystick2 = new CommandXboxController(1); // Secondary

  //Create Subsystems
  public static final CommandSwerveDrivetrain m_Drivetrain = TunerConstants.DriveTrain; // My drivetrain  
  public static final Intake m_Intake = new Intake();
  public static final Ampinator m_Trampinator = new Ampinator();
  public static final AmpElevator m_TrampElevator = new AmpElevator();
  public static final Shooter m_Shooter = new Shooter();
  public static final Climber m_Climber = new Climber();
  public static final Limelight m_Limelight = new Limelight();
  public static final LimelightRear m_LimelightRear = new LimelightRear();
  public static final Blinkin m_Blinkin = new Blinkin();
  public static final CANdleSub m_Candlesub = new CANdleSub();
  public static final ShooterAngle m_ShooterAngle = new ShooterAngle();

  public static GenericEntry shooterPower;
  

  //Drive Swerve
  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
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
            .withRotationalRate(joystick.a().getAsBoolean()?m_Limelight.LimelightAim():-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    
    m_Candlesub.setDefaultCommand(new AutoLEDTarget(m_Candlesub));
    m_ShooterAngle.setDefaultCommand(new AutoShooterAngle(m_ShooterAngle));
    

    //Button Bindings
    
    // reset the field-centric heading
    joystick.start().onTrue(m_Drivetrain.runOnce(() -> m_Drivetrain.seedFieldRelative()));

    /* Intake Commands */
    joystick2.a().onTrue(new ShooterIntake(m_Intake, m_Shooter).withTimeout(5)
      .andThen(new GotIt().withTimeout(.1))
    );
    
    joystick2.b().onTrue(new AmpIntake(m_Intake, m_Trampinator).withTimeout(3).andThen(new GotIt().withTimeout(.1)));
    
    //Manual Amp Control
    joystick2.back().whileTrue(new AmpIntakeManual(m_Intake, m_Trampinator, -1));
    joystick2.back().onFalse(new AmpIntakeManual(m_Intake, m_Trampinator, 0));
    joystick2.start().whileTrue(new AmpIntakeManual(m_Intake, m_Trampinator, 1));
    joystick2.start().onFalse(new AmpIntakeManual(m_Intake, m_Trampinator, 0)); 
    
    // Manual Shooter Intake Control
    joystick2.povRight().whileTrue(new AutoIntake(m_Intake));
    joystick2.povRight().onFalse(new StopAllShooter(m_Intake, m_Shooter));

    joystick2.povLeft().whileTrue(new ReverseIntake(m_Intake));
    joystick2.povLeft().onFalse(new StopAllShooter(m_Intake, m_Shooter));

    /*Ampinator Commands */
    joystick2.x().whileTrue(new InstantCommand(() -> m_Trampinator.runShooterSpeed(1)));
    joystick2.x().onFalse(new InstantCommand(() -> m_Trampinator.runShooterSpeed(0)));

    joystick2.y().whileTrue(new InstantCommand(() -> m_Trampinator.runShooterSpeed(-1)));
    joystick2.y().onFalse(new InstantCommand(() -> m_Trampinator.runShooterSpeed(0)));

    /*Amp Elevator Commands */
    joystick2.rightBumper().onTrue(m_TrampElevator.setElevatorGoalCommand(0.34));
    joystick2.leftBumper().onTrue(m_TrampElevator.setElevatorGoalCommand(0.0));

    /*Shooter Commands */
    joystick.b().whileTrue(new InstantCommand(() -> m_Shooter.shootClose(1250)));
    joystick.b().onFalse(new InstantCommand(() -> m_Shooter.stopShooter()));

    joystick.x().onTrue(new AutoShooter(m_Shooter).withTimeout(1).andThen(new ParallelCommandGroup(
      new AutoFeeder(m_Intake),
      new AutoShooter(m_Shooter)
    ).withTimeout(.5).andThen(new AutoStopShooter().withTimeout(.03))));

    /*Climber Commands */
    joystick.rightBumper().whileTrue(new InstantCommand(() -> m_Climber.windUp(1.0 )));
    joystick.rightBumper().onFalse(new InstantCommand(() -> m_Climber.windUp(0)));
    
    joystick.leftBumper().whileTrue(new InstantCommand(() -> m_Climber.windUp(-1.0)));
    joystick.leftBumper().onFalse(new InstantCommand(() -> m_Climber.windUp(0)));

    /*Smart Dashboard Commands*/

    //Shooter Tuner
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter Tuner");
    ShuffleboardLayout shooterCommands = tab
      .getLayout("Shooter", BuiltInLayouts.kList)
      .withSize(2,2);
      
    GenericEntry rpm = shooterCommands.add("Shooter RPM", 0).getEntry();
    shooterCommands.add("Enable Shooter", new InstantCommand(() -> m_Shooter.shoot(rpm.getDouble(0))));
    shooterCommands.add("Enable Feeder", new AutoFeeder(m_Intake));
    shooterCommands.add("Disable Shooter", new InstantCommand(() -> m_Shooter.stopShooter()));
    GenericEntry angle = shooterCommands.add("Shooter Angle", 0).getEntry();
    shooterCommands.add("Change Shooter Angle", new InstantCommand(() -> m_ShooterAngle.setShooterAngle(angle.getDouble(0))));

    //Shooter Adjust for game piece newness
    shooterPower = tab.add("Shooter Percent", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("Min", 0, "Max", 1))
      .getEntry();

    if (Utils.isSimulation()) {
      m_Drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_Drivetrain.registerTelemetry(logger::telemeterize); 
  }

  public RobotContainer() {
    //Auto Naming of Commands and Such//
    NamedCommands.registerCommand("ShooterIntake", new ShooterIntake(m_Intake, m_Shooter));
    NamedCommands.registerCommand("IntakeBump", new AutoFeeder(m_Intake).withTimeout(.01));
    NamedCommands.registerCommand("AutoShooter", new AutoShooter(m_Shooter).withTimeout(.75));
    NamedCommands.registerCommand("AutoPooper", new AutoSlowShoot(m_Shooter));
    NamedCommands.registerCommand("AutoFeeder", new AutoFeeder(m_Intake).withTimeout(.75));
    NamedCommands.registerCommand("PooperFeeder", new AutoFeeder(m_Intake));
    NamedCommands.registerCommand("AutoStopShooter", new AutoStopShooter().withTimeout(.03));
    NamedCommands.registerCommand("PathStopPooper", new StopAllShooter(m_Intake, m_Shooter).withTimeout(.03));
    
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