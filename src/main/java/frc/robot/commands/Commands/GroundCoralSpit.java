package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.GroundCoral;
import edu.wpi.first.wpilibj2.command.Command;

public class GroundCoralSpit extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private GroundCoral GroundCoral;
  

  public GroundCoralSpit(GroundCoral subsystem1) {
    GroundCoral = subsystem1;
    addRequirements(subsystem1);
}
@Override
  public void initialize() {
    GroundCoral.setSpeed(-1 * Constants.GroundCoralMult);
    }

  @Override
  public void execute() {
    GroundCoral.setSpeed(-1 * Constants.GroundCoralMult);
  }
  @Override
  public void end(boolean interrupted) {
    GroundCoral.setSpeed(0);

  }
  @Override
  public boolean isFinished() {
    return false;
  }
}