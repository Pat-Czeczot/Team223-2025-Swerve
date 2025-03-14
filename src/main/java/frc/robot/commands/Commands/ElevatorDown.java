package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDown extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Elevator elevator;
  private double stickPos;
  //private Elevator motor2;

  public ElevatorDown(Elevator subsystem1, double stickPos) {
    elevator = subsystem1;
    this.stickPos = stickPos;
    addRequirements(subsystem1);
    
}
@Override
  public void initialize() {
    }

  @Override
  public void execute() {
    elevator.setSpeed(stickPos * Constants.elevatorMult);
  }
  @Override
  public void end(boolean interrupted) {
    elevator.setSpeed(0);
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}
