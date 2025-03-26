package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
    
  private SparkFlex Wrist;
  private DigitalInput wristSensor;
  private RelativeEncoder wristEncoder;

  public Wrist() {
    Wrist = new SparkFlex(Constants.WristID, MotorType.kBrushless);
    wristEncoder = Wrist.getExternalEncoder();
    wristEncoder.setPosition(0);

    wristSensor = new DigitalInput(2); //Port 2 DIO
    Wrist.setInverted(true);
    //Wrist.burnFlash();
  }
    public void setSpeed(double speed){
    //System.out.println(!wristSensor.get());


    if (speed < 0 && !wristSensor.get()) 
    {
      Wrist.set(0);
      wristEncoder.setPosition(0);
    }
    else if (speed > 0 && wristEncoder.getPosition() >= 1.00)
    {
      Wrist.set(speed * 0);
    }
    else
    {
      //System.out.println(wristEncoder.getPosition());

      Wrist.set(speed);
      
    }
}
  
  @Override
  public void periodic() {
  }
  @Override
  public void simulationPeriodic() {
  }
}