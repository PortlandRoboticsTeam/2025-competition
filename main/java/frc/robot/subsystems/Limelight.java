package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
  NetworkTableInstance tables = NetworkTableInstance.getDefault();
  NetworkTable right = tables.getTable("limelight");
  NetworkTable left = tables.getTable("limelight");
  boolean useRight = true;

  public void setlimeLight(boolean useRight){
    this.useRight = useRight;
  }
  public double getx() {
    double p = .06;
    return -(useRight?right:left).getEntry("ty").getDouble(0)*p;
  }

  public double gety() {
    double p = .1;
    return (useRight?right:left).getEntry("tx").getDouble(0)*p;
  }

  @Override
  public void periodic() {
      super.periodic();
      SmartDashboard.putNumber("vis out", (useRight?right:left).getEntry("tx").getDouble(0));
  }
}
