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
    DigitalInput beamSensor = new DigitalInput(3);

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
      RobotContainer.updateLEDS(beamSensor.get());
    }
    @Override
    public void simulationPeriodic() {
    }
  }