package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

/**
 * 
 * Add your docs here.
 */
public class Limelight extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  NetworkTable table;
  NetworkTableEntry tx, ty, tv, ta, ts;
  double x, y, v, area, s, limelightMountAngleDegrees, limelightLensHeightInches, goalHeightInches, angleToGoalDegrees, angleToGoalRadians, distanceFromLimelightToGoalInches, kP;

  public Limelight(){
    table = NetworkTableInstance.getDefault().getTable("limelight-front");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    ta = table.getEntry("ta");
    ts = table.getEntry("ts");
    
    limelightMountAngleDegrees = 5.0; 
    limelightLensHeightInches = 20.0; 
    goalHeightInches = 57.0; 
    
  }


  public void readValues(){
     x = tx.getDouble(0.0);
     y = ty.getDouble(0.0);
     v = tv.getDouble(0.0);
     area = ta.getDouble(0.0);
     s = ts.getDouble(0.0);
    //postToDashboard();
  }

  
  public void postToDashboard(){
    SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightV", v);
    //SmartDashboard.putNumber("LimelightArea", area);
    //SmartDashboard.putNumber("LimelightSkew", s);
    //SmartDashboard.putNumber("LEDMODE", (int)table.getEntry("ledMode").getDouble(0.0));
  }
  
  public boolean isAimed(){
    return getX()<=1 && getX()>=-1; 
  }


  public double getX(){
    readValues();
    return x;
  }

  public double getY(){
    readValues();
    return y;
  }

  public double getV(){
    readValues();  
    return v;
  }

  public double getArea(){
    readValues();
    return area;
  }

  public double getSkew(){
    readValues();
    return s;
  }

  public boolean seesTarget(){
    readValues(); 
    if( v == 0 )
      return false;
    else  
      return true;
  }

  //a value of 0 represents the vision processor
  //a value of 1 represents the driver camera
  public void setView(int n){
    table.getEntry("camMode").setNumber(n);
  }

  public int getView() {
    return (int)table.getEntry("camMode").getDouble(0.0);
  }

  //0 is side by side, 1 is PiP Main with limelight big
  //2 is PiP Secondary with Secondary big
  public void setStreamMode(int n ){
    table.getEntry("stream").setNumber(n);
  }

  public int getStreamMode() {
    return (int)table.getEntry("stream").getDouble(0.0);
  }

  //0: use pipeline default
  //1: force off, 2: force blink, 3: force on
  public void setLedMode(int n  ){
    table.getEntry("ledMode").setNumber(n);
  }

  public void setPipeline( int p){
    table.getEntry("pipeline").setNumber(p);
  }

  public double getDistance(){
      angleToGoalDegrees = limelightMountAngleDegrees + getY();
      angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
      distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
      return distanceFromLimelightToGoalInches;
  }

  public double LimelightAim()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    if(getX() > -3 && getX() < 3){
      kP = .17;
    } else {
      kP = .0175;
    }
    
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = getX() * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= RobotContainer.MaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

}
