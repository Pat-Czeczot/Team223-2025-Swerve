package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class CoralIntake extends SubsystemBase {
    
    SparkMax leftflywheel;
    SparkMax rightflywheel;
    public DigitalInput beamSensor = new DigitalInput(3);

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

      public void setSpeed(double speed1, double speed2)
      {
        leftflywheel.set(speed1);
        rightflywheel.set(speed2);
      }
    @Override
    public void periodic() {
      RobotContainer.updateLEDS(beamSensor.get());
    }
    @Override
    public void simulationPeriodic() {
    }
  }