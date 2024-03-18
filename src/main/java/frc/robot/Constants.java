package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class IntakeConstants {
      public static final int intakeLeft = 2;
      public static final int intakeRight = 1;
      public static final int intakeChooser = 8;
      public static final int shooterFeeder = 7;
    }

    public static final class ShooterConstants {
      public static final int shooterLeft = 4;
      public static final int shooterRight = 5;
      public static final int shooterAngle = 6;
      

      public static final double kAngleGearRatio = 5.0 * 5.0 * 10.0;  

      // PID coefficients for Shooter
      public static final double KP = .0006; 
      public static final double KI = 0.00000;
      public static final double KD = 0; 
      public static final double KIz = 0; 
      public static final double KFF = 0.000157; 
      public static final double KMaxOutput = 1; 
      public static final double KMinOutput = -1;
      public static final double MaxRPM = 6700;

      // PID coefficients for Shooter Angle Motor
      public static final double ANGLEKP = 11.0; //0.65
      public static final double ANGLEKI = 0.00000;
      public static final double ANGLEKD = 0; 
      public static final double ANGLEKIz = 0; 
      public static final double ANGLEKFF = 0.0025; 
      public static final double ANGLEKMaxOutput = 1; 
      public static final double ANGLEKMinOutput = -1;
      public static final double ANGLEMaxRPM = 6700;   
    }

    public static final class TrampinatorConstants {

      public static final int elevator = 10;
      public static final int shooter = 9;

      public static final double kGearRatio = 9.0;
      public static final double kMeterPerRevolution = Units.inchesToMeters((1.888*Math.PI)/kGearRatio);

      // PID coefficients for AMP/Trap
      public static final double KP = 3.0; 
      public static final double KI = 0.000000;
      public static final double KD = 0.0000; 
      public static final double KIz = 0.0; 
      public static final double KFF = 0.3;
      public static final double MaxOutput = 1.0; 
      public static final double KMinOutput = -1.0;
      public static final double MaxRPM = 5700;
    }

    public static final class ClimberConstants {
      public static final int captainHook = 3;
    }

    public static final class BlinkinConstants {
      public static final int led = 0;
    }
}
