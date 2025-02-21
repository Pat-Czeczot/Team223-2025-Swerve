package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDown extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Climber Climber;
  

  public ClimberDown(Climber subsystem1) {
    Climber = subsystem1;
    addRequirements(subsystem1);
}
@Override
  public void initialize() {
    Climber.setSpeed(-1 * Constants.ClimberMult);
    }

  @Override
  public void execute() {
    Climber.setSpeed(-1 * Constants.ClimberMult);
  }
  @Override
  public void end(boolean interrupted) {
    Climber.setSpeed(0);

  }
  @Override
  public boolean isFinished() {
    return false;
  }
}