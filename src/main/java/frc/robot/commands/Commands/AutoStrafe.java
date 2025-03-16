package frc.robot.commands.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.DriveSubsystem;

public class AutoStrafe extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private DriveSubsystem swerve;
    private CoralIntake coralIntake;
    private boolean isTripped = false;

    public AutoStrafe(DriveSubsystem drivetrain, CoralIntake coral) {
        swerve = drivetrain;
        coralIntake = coral;
        addRequirements(swerve);
        addRequirements(coral);
    }

    @Override
  public void initialize() {
    isTripped = false;
    }

  @Override
  public void execute() {
    if (!coralIntake.beamSensor.get() && !isTripped)
    {
        swerve.drive(0, 0.1, 0, false);
    }
    else
    {
        swerve.drive(0, 0, 0, false);
        isTripped = true;
    }
    
    
  }
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0, false);
  }
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
