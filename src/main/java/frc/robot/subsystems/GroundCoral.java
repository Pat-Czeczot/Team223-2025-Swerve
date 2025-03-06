package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundCoral extends SubsystemBase {
    
  SparkMax GroundCoral;
 

  public GroundCoral() {
    GroundCoral = new SparkMax(Constants.GroundCoralID, MotorType.kBrushless);
    GroundCoral.setInverted(true);
    //GroundCoral.burnFlash();
  }
    public void setSpeed(double speed){
      GroundCoral.set(speed);
  }
  @Override
  public void periodic() {
  }
  @Override
  public void simulationPeriodic() {
  }
}