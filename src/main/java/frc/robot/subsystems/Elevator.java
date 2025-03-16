package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

<<<<<<< HEAD
import edu.wpi.first.math.MathUtil;
=======
>>>>>>> 804e25247023b76009fec1c3cde469094a22825a
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    
    private SparkFlex motor1;
    private SparkFlex motor2;
    private DigitalInput lowSensor;
    private DigitalInput highSensor;
    private RelativeEncoder encoder;
<<<<<<< HEAD
    private PIDController pid;
    
=======
    private SparkClosedLoopController elevatorPID;
    private PIDController PID = new PIDController(0.5, 0, 0);
>>>>>>> 804e25247023b76009fec1c3cde469094a22825a

    SparkFlexConfig elevatorconfig1 = new SparkFlexConfig();
    SparkFlexConfig elevatorconfig2 = new SparkFlexConfig();
    double tolerance = 0.05;
    double position;

    public Elevator() {
        motor1 = new SparkFlex(Constants.motor1ID, MotorType.kBrushless);
        motor2 = new SparkFlex(Constants.motor2ID, MotorType.kBrushless);
        encoder = motor1.getExternalEncoder();
        encoder.setPosition(0);
        elevatorPID = motor1.getClosedLoopController();
        
        elevatorconfig1.closedLoop.pid(2.3, 0, 0);
        elevatorconfig1.inverted(true);
        elevatorconfig2.follow (motor1);
        //elevatorconfig2.inverted(true);
       // motor1.configure (elevatorconfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //motor2.configure (elevatorconfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        lowSensor = new DigitalInput(0); //Port 0 DIO
        highSensor = new DigitalInput(1); //Port 1 DIO
<<<<<<< HEAD
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
=======
      
    }
    public void setSpeed(double speed){
        System.out.println("Encoder Position: " + encoder.getPosition());
        //System.out.println(!highSensor.get());
>>>>>>> 804e25247023b76009fec1c3cde469094a22825a
        

         if (speed < 0 && !lowSensor.get()) 
        {
          motor1.set(0);
          motor2.set(0);
          encoder.setPosition(0);
        }
        else if (speed < 0 && encoder.getPosition() <=2.50)
        {
          motor1.set(speed * 0.30);
<<<<<<< HEAD
          motor2.set(speed * 0.30);  

=======
          motor2.set(speed * 0.30);
>>>>>>> 804e25247023b76009fec1c3cde469094a22825a
        }
        else if (speed > 0 && !highSensor.get())
        {
          motor1.set(0);
          motor2.set(0);
        }
        else if (speed > 0 && encoder.getPosition() >= 7.00)
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
<<<<<<< HEAD

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
=======
    public void moveTo(int pos) {
      int index = pos - 1;
      double[] positions = {1.0, 2.9, 5.2, 9.9};
     // elevatorPID.setReference(pos, ControlType.kPosition);
     //System.out.println("PID Output: " + PID.calculate(encoder.getPosition(), pos));

    //setSpeed(PID.calculate(encoder.getPosition(), pos));
       if (positions[index] - encoder.getPosition() > 0.08)
>>>>>>> 804e25247023b76009fec1c3cde469094a22825a
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
<<<<<<< HEAD
      */
      
=======
  
>>>>>>> 804e25247023b76009fec1c3cde469094a22825a
    }
    @Override
    public void periodic() {
      SmartDashboard.putNumber("elevatorposition", encoder.getPosition());
      SmartDashboard.putBoolean("elevatoratposition", isAtPosition());
      SmartDashboard.putNumber("elevatorsetpoint", position);
    }
    @Override
    public void simulationPeriodic() {
    }
    public boolean isAtPosition(){
      return Math.abs(position - encoder.getPosition())<tolerance;
    }
  }