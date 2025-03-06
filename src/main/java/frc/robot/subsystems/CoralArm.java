package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralArm extends SubsystemBase {
    
  SparkFlex CoralArm;
 

  public CoralArm() {
    CoralArm = new SparkFlex(Constants.CoralArmID, MotorType.kBrushless);
    CoralArm.setInverted(true);
    //CoralArm.burnFlash();
  }
    public void setSpeed(double speed){
      CoralArm.set(speed);
  }
  @Override
  public void periodic() {
  }
  @Override
  public void simulationPeriodic() {
  }
}