package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    
    private SparkFlex motor1;
    private SparkFlex motor2;
    private DigitalInput lowSensor;
    private DigitalInput highSensor;
    private RelativeEncoder encoder;
    private PIDController pid;
    


    public Elevator() {
        motor1 = new SparkFlex(Constants.motor1ID, MotorType.kBrushless);
        motor2 = new SparkFlex(Constants.motor2ID, MotorType.kBrushless);
        encoder = motor1.getExternalEncoder();
        encoder.setPosition(0);

        lowSensor = new DigitalInput(0); //Port 0 DIO
        highSensor = new DigitalInput(1); //Port 1 DIO
        motor1.setInverted(true);
        motor2.setInverted(true);
        pid = new PIDController(0.5, 0, 0);
      //motor1.burnFlash();
      //motor2.burnFlash();
      //10.4
    }
    public void setSpeed(double speed){
        //System.out.println(encoder.getPosition());
       // System.out.println(!highSensor.get());
        

         if (speed < 0 && !lowSensor.get()) 
        {
          motor1.set(0);
          motor2.set(0);
          encoder.setPosition(0);
        }
        else if (speed < 0 && encoder.getPosition() <=2.50)
        {
          motor1.set(speed * 0.30);
          motor2.set(speed * 0.30);  

        }
        else if (speed > 0 && !highSensor.get())
        {
          motor1.set(0);
          motor2.set(0);
        }
        else if (speed > 0 && encoder.getPosition() >= 9.00)
        {
          motor1.set(speed * 0.30);
          motor2.set(speed * 0.30);
        }
        else
        {
          motor1.set(speed);
          motor2.set(speed);
        }
    }

    private void setSpeedAuto(double speed){
     // System.out.println(encoder.getPosition());
     // System.out.println(!highSensor.get());
      

       if (speed < 0 && !lowSensor.get()) 
      {
        motor1.set(0);
        motor2.set(0);
        encoder.setPosition(0);
      }
      else if (speed > 0 && !highSensor.get())
      {
        motor1.set(0);
        motor2.set(0);
      }
      else
      {
        motor1.set(speed);
        motor2.set(speed);
      }
  }
    
    public void moveTo(double pos) {
      //int index = pos - 1;
      //double[] positions = {1.0, 2.5, 5.7, 9.9};

      setSpeedAuto(MathUtil.clamp(pid.calculate(encoder.getPosition(), pos), -1, 1));

       
     /*  if (pos - encoder.getPosition() > 0.08)
      {
        setSpeed(1);
      }
      else if (encoder.getPosition() - pos > 0.08)
      {
        setSpeed(-1);
      }
      else
      {
        setSpeed(0);
      }
      */
      
    }
    @Override
    public void periodic() {
    }
    @Override
    public void simulationPeriodic() {
    }
  }