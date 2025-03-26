package frc.robot.commands.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralIntake; 
import edu.wpi.first.wpilibj2.command.Command;

public class L1CoralSpit extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private CoralIntake coralIntake;
  private double speedLeft, speedRight;

  public L1CoralSpit(CoralIntake subsystem1, double speedLeft, double speedRight) { 
    coralIntake = subsystem1;
    this.speedLeft = speedLeft;
    this.speedRight = speedRight;
    addRequirements(subsystem1); 
}
@Override
  public void initialize() {
    }

  @Override
  public void execute() {
    coralIntake.setSeperateSpeeds(speedLeft,  speedRight);
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