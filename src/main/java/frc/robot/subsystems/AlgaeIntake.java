package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
    
  SparkFlex algaeintake;

  public AlgaeIntake() {
    algaeintake = new SparkFlex(Constants.algaeintakeID, MotorType.kBrushless);
    algaeintake.setInverted(true);
    //algaeintake.burnFlash();
  }
    public void setSpeed(double speed){
      algaeintake.set(speed);
  }
  @Override
  public void periodic() {
  }
  @Override
  public void simulationPeriodic() {
  }
}