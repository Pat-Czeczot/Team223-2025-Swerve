package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralIntake; 
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntakeSuck extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private CoralIntake coralIntake;

  public CoralIntakeSuck(CoralIntake subsystem1) { 
    coralIntake = subsystem1;
    addRequirements(subsystem1); 
}
@Override
  public void initialize() {
    }

  @Override
  public void execute() {
    coralIntake.setSpeed(1 * Constants.flywheelsMult);
  }
  @Override
  public void end(boolean interrupted) {
    coralIntake.setSpeed(0);

  }
  @Override
  public boolean isFinished() {
    return false;
  }
}