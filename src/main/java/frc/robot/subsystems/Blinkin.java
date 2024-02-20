package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class Blinkin extends SubsystemBase {
  /** Creates a new Blinkin2. */

    public static Spark blinkin;

    private static Blinkin m_Blinkin = null;

    public double index = -0.15;

  public Blinkin() {

   blinkin = new Spark(Constants.BlinkinConstants.led);
  }

  @Override
  public void periodic() {
  
  }


  public void orange() {
    blinkin.set(0.65);
  }

  public void green() {
    blinkin.set(0.27);
  }

  public void random() {
    blinkin.set(0.30);
  }

   public void defult() {
    blinkin.set(-.29);
  }




}