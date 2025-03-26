/* 
package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DrivetrainRoutine {

    private final SysIdRoutine sysIdRoutine;

    // Mutable holders for unit-safe voltage, linear distance, and linear velocity values, persisted to avoid reallocation.
    private final MutVoltage driveRoutineAppliedVoltage = Volts.mutable(0);
    private final MutDistance driveRoutineDistance = Meters.mutable(0);
    private final MutLinearVelocity driveRoutineVelocity = MetersPerSecond.mutable(0);

    public DrivetrainRoutine(DriveSubsystem drivetrain) {
        // SysID characterization configuration
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Units.Second), // Ramp rate in V/s
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                Time.ofBaseUnits(7, Units.Second), // Use default timeout of 10 sec
                null
            ),
            new SysIdRoutine.Mechanism(
                // Tell SysID how to plumb the driving voltage to the motors
                voltage -> {
                    //drivetrain.setModulesToZeroRadPID(); //Sets the rotational pid to zero, prob not needed
                    drivetrain.m_frontLeft.m_drivingSpark.setVoltage(voltage.in(Volts));
                    drivetrain.m_frontRight.m_drivingSpark.setVoltage(voltage.in(Volts));
                    drivetrain.m_rearLeft.m_drivingSpark.setVoltage(voltage.in(Volts));
                    drivetrain.m_rearRight.m_drivingSpark.setVoltage(voltage.in(Volts));
                }, 
                // Tell SysID how to record a frame of data for each motor on the mechanism
                log -> {
                    log.motor("frontleft")
                        .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.m_frontLeft.getDriveVolts(), Volts))
                        .linearPosition(driveRoutineDistance.mut_replace(drivetrain.m_frontLeft.m_drivingEncoder.getPosition(), Meters))
                        .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.m_frontLeft.m_drivingEncoder.getVelocity(), MetersPerSecond));

                    log.motor("frontright")
                        .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.m_frontRight.getDriveVolts(), Volts))
                        .linearPosition(driveRoutineDistance.mut_replace(drivetrain.m_frontRight.m_drivingEncoder.getPosition(), Meters))
                        .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.m_frontRight.m_drivingEncoder.getVelocity(), MetersPerSecond));

                    log.motor("rearleft")
                        .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.m_rearLeft.getDriveVolts(), Volts))
                        .linearPosition(driveRoutineDistance.mut_replace(drivetrain.m_rearLeft.m_drivingEncoder.getPosition(), Meters))
                        .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.m_rearLeft.m_drivingEncoder.getVelocity(), MetersPerSecond));

                    log.motor("rearright")
                        .voltage(driveRoutineAppliedVoltage.mut_replace(drivetrain.m_rearRight.getDriveVolts(), Volts))
                        .linearPosition(driveRoutineDistance.mut_replace(drivetrain.m_rearRight.m_drivingEncoder.getPosition(), Meters))
                        .linearVelocity(driveRoutineVelocity.mut_replace(drivetrain.m_rearRight.m_drivingEncoder.getVelocity(), MetersPerSecond));
                }, 
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name
                drivetrain
            )
        );
    }

    // Returns a command that will execute a quasistatic test in the given direction
    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }
    public Command quasistaticReverse() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    // Returns a command that will execute a dynamic test in the given direction
    public Command dynamicForward() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }
    public Command dynamicReverse() {
        return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
*/