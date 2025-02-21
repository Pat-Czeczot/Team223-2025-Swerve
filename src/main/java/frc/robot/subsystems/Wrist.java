package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
    
  SparkFlex Wrist;

  public Wrist() {
    Wrist = new SparkFlex(Constants.WristID, MotorType.kBrushless);
    Wrist.setInverted(true);
    //Wrist.burnFlash();
  }
    public void setSpeed(double speed){
        Wrist.set(speed);
  }
  @Override
  public void periodic() {
  }
  @Override
  public void simulationPeriodic() {
  }
}