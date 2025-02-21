package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralIntake; 
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntakeSuck extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private CoralIntake leftflywheel;
  private CoralIntake rightflywheel;

  public CoralIntakeSuck(CoralIntake subsystem1) { 
    leftflywheel = subsystem1;
    rightflywheel = subsystem1;
    addRequirements(subsystem1); 
}
@Override
  public void initialize() {
    }

  @Override
  public void execute() {
    leftflywheel.setSpeed(-1 * Constants.leftflywheelMult);
    rightflywheel.setSpeed(-1 * Constants.rightflywheelMult);
  }
  @Override
  public void end(boolean interrupted) {
    leftflywheel.setSpeed(0);
    rightflywheel.setSpeed(0);

  }
  @Override
  public boolean isFinished() {
    return false;
  }
}