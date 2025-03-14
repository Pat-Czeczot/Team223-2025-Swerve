package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorToPos extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Elevator elevator;
  private int position;

  public ElevatorToPos(Elevator subsystem1, int pos) {
    elevator = subsystem1;
    position = pos;
    addRequirements(subsystem1);
    
}
@Override
public void initialize() {
  elevator.moveTo(position);
}

@Override
  public void execute() {
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