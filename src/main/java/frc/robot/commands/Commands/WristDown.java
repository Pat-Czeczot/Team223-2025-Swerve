package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;

public class WristDown extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Wrist Wrist;


  public WristDown(Wrist subsystem1) {
    Wrist = subsystem1;
    addRequirements(subsystem1);
}
@Override
  public void initialize() {
    Wrist.setSpeed(-1 * Constants.WristMult);
    }

  @Override
  public void execute() {
    Wrist.setSpeed(-1 * Constants.WristMult);
  }
  @Override
  public void end(boolean interrupted) {
    Wrist.setSpeed(0);
    
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}