package frc.robot;

/** Add your docs here. */
public class Constants {

    public static final class IntakeConstants {
        public static final int intakeLeft = 1;
        public static final int intakeRight = 2;
        public static final int intakeChooser = 3;
     }

     public static final class ShooterConstants {
        public static final int shooterLeft = 4;
        public static final int shooterRight = 5;
        public static final int shooterAngle = 6;
        public static final int shooterFeeder = 7;
 
 
             // PID coefficients for Shooter
         public static final double KP = 0.0000; 
         public static final double KI = 0.00000;
         public static final double KD = 0; 
         public static final double KIz = 0; 
         public static final double KFF = 0.0003; 
         public static final double KMaxOutput = 1; 
         public static final double KMinOutput = -1;
         public static final double MaxRPM = 5700;
 
         
 
      }

      public static final class TrampinatorConstants {

        public static final int elevator = 8;
        public static final int shooter = 9;
  
        public static final double kGearRatio = 5.0;
  
          // PID coefficients for AMP/Trap
        public static final double KP = 1.0; 
        public static final double KI = 0.000000;
        public static final double KD = 0.0000; 
        public static final double KIz = 0.0; 
        public static final double KFF = 0.0004;
        public static final double MaxOutput = 1.0; 
        public static final double KMinOutput = -1.0;
        public static final double MaxRPM = 5700;

        public static final double kSVolts = 0.14;
        public static final double kGVolts = 0.02;
        public static final double kVVoltSecondPerRad = 4.65;
        public static final double kAVoltSecondSquaredPerRad = 0.04;

        public static final double kMaxVelocityRadPerSecond = .5;
        public static final double kMaxAccelerationRadPerSecSquared = 1.0;
    

       }

       public static final class ClimberConstants {
        public static final int captainHook = 10;
        public static final int climbing = 11;

        

         // PID coefficients for Climber
         public static final double climbingCenterKP = 0; 
         public static final double climbingCenterKI = 0.000000;
         public static final double climbingCenterKD = 0.0000; 
         public static final double climbingCenterKIz = 0; 
         public static final double climbingCenterKFF = 0.0004;
         public static final double climbingCenterKMaxOutput = 1; 
         public static final double climbingCenterKMinOutput = -1;
         public static final double climbingCenterMaxRPM = 5700;
                 

     }


    

}
