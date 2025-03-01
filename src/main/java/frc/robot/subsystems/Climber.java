package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    
  SparkFlex Climber;
 

  public Climber() {
    Climber = new SparkFlex(Constants.ClimberID, MotorType.kBrushless);
    Climber.setInverted(true);
    //Climber.burnFlash();
  }
    public void setSpeed(double speed){
        Climber.set(speed);
  }
  @Override
  public void periodic() {
  }
  @Override
  public void simulationPeriodic() {
  }
}