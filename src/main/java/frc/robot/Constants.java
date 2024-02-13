package frc.robot;

/** Add your docs here. */
public class Constants {

    public static final class IntakeConstants {
        public static final int intakLeft = 1;
        public static final int intakeRight = 2;
        public static final int intakeChooser = 3;
     }

     public static final class ShooterConstants {
        public static final int shooterLeft = 4;
        public static final int shooterRight = 5;
        public static final int shooterAngle = 6;
        public static final int shooterFeeder = 7;
 
 
             // PID coefficients for Shooter
         public static final double shooterAngleKP = 0.0000; 
         public static final double shooterAngleKI = 0.00000;
         public static final double shooterAngleKD = 0; 
         public static final double shooterAngleKIz = 0; 
         public static final double shooterAngleKFF = 0.0003; 
         public static final double shooterKMaxOutput = 1; 
         public static final double shooterAngleKMinOutput = -1;
         public static final double shooterAngleMaxRPM = 5700;
 
         
 
      }

      public static final class TrampinatorConstants {

        public static final int elevator = 8;
        public static final int shooter = 9;
  
  
  
          // PID coefficients for AMP/Trap
          public static final double elevatorLeftKP = 0; 
          public static final double elevatorLeftKI = 0.000000;
          public static final double elevatorLeftKD = 0.0000; 
          public static final double elevatorLeftKIz = 0; 
          public static final double elevatorLeftKFF = 0.0004;
          public static final double elevatorLeftKMaxOutput = 1; 
          public static final double elevatorLeftKMinOutput = -1;
          public static final double elevatorLeftMaxRPM = 5700;
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
