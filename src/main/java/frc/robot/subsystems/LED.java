package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LED extends SubsystemBase {
private double Color = 0.0;
private Spark ledController = new Spark(0);

@Override
  public void periodic() {
    ledController.set(0.77);
  }

  



}  