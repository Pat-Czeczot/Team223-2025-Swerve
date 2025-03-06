package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralArm;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralArmDown extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private CoralArm CoralArm;
  

  public CoralArmDown(CoralArm subsystem1) {
    CoralArm = subsystem1;
    addRequirements(subsystem1);
}
@Override
  public void initialize() {
    CoralArm.setSpeed(-1 * Constants.CoralArmMult);
    }

  @Override
  public void execute() {
    CoralArm.setSpeed(-1 * Constants.CoralArmMult);
  }
  @Override
  public void end(boolean interrupted) {
    CoralArm.setSpeed(0);

  }
  @Override
  public boolean isFinished() {
    return false;
  }
}