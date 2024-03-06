package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Blinkin extends SubsystemBase {
  /** Creates a new Blinkin2. */

    public static Spark blinkin;

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
    blinkin.set(.91);
  }

  public void red() {
    blinkin.set(.61);
  }

  public void blue() {
    blinkin.set(.87);
  }




}