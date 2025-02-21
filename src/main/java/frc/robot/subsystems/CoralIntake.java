package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntake extends SubsystemBase {
    
    SparkMax leftflywheel;
    SparkMax rightflywheel;

    public CoralIntake() {
        leftflywheel = new SparkMax(Constants.leftflywheelID, MotorType.kBrushless);
      rightflywheel = new SparkMax(Constants.rightflywheelID, MotorType.kBrushless);
      leftflywheel.setInverted(true);
      rightflywheel.setInverted(false);
      //leftflywheel.burnFlash();
      //rightflywheel.burnFlash();
    }
      public void setSpeed(double speed){
        leftflywheel.set(speed);
        rightflywheel.set(speed);
    }
    @Override
    public void periodic() {
    }
    @Override
    public void simulationPeriodic() {
    }
  }