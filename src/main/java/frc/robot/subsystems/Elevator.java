package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    
    SparkFlex motor1;
    SparkFlex motor2;

    public Elevator() {
        motor1 = new SparkFlex(Constants.motor1ID, MotorType.kBrushless);
        motor2 = new SparkFlex(Constants.motor2ID, MotorType.kBrushless);
        motor1.setInverted(true);
        motor2.setInverted(true);
      //motor1.burnFlash();
      //motor2.burnFlash();
    }
      public void setSpeed(double speed){
        motor1.set(speed);
        motor2.set(speed);
    }
    @Override
    public void periodic() {
    }
    @Override
    public void simulationPeriodic() {
    }
  }