package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDown extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Elevator motor1;
  private Elevator motor2;

  public ElevatorDown(Elevator subsystem1) {
    motor1 = subsystem1;
    motor2 = subsystem1;
    addRequirements(subsystem1);
    
}
@Override
  public void initialize() {
    }

  @Override
  public void execute() {
    motor1.setSpeed(-1 * Constants.elevatorMult);
    motor2.setSpeed(-1 * Constants.elevatorMult);
  }
  @Override
  public void end(boolean interrupted) {
    motor1.setSpeed(0);
    motor2.setSpeed(0);

  }
  @Override
  public boolean isFinished() {
    return false;
  }
}