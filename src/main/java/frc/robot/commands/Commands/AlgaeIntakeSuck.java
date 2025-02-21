package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.AlgaeIntake;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeIntakeSuck extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private AlgaeIntake algaeintake;


  public AlgaeIntakeSuck(AlgaeIntake subsystem1) {
    algaeintake = subsystem1;
    addRequirements(subsystem1);
}
@Override
  public void initialize() {
    algaeintake.setSpeed(-1 * Constants.algaeintakeMult);
    }

  @Override
  public void execute() {
    algaeintake.setSpeed(-1 * Constants.algaeintakeMult);
  }
  @Override
  public void end(boolean interrupted) {
    algaeintake.setSpeed(0);
    
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}