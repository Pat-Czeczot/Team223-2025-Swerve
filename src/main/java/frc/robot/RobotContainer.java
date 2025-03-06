// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import pabeles.concurrency.IntObjectTask;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

//Subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.GroundCoral;


//Commands
import frc.robot.commands.Commands.AlgaeIntakeSpit;
import frc.robot.commands.Commands.AlgaeIntakeSuck;
import frc.robot.commands.Commands.CoralIntakeSpit;
import frc.robot.commands.Commands.CoralIntakeSuck;
import frc.robot.commands.Commands.ElevatorUp;
import frc.robot.commands.Commands.ElevatorDown;
import frc.robot.commands.Commands.CoralArmUp;
import frc.robot.commands.Commands.CoralArmDown;
import frc.robot.commands.Commands.GroundCoralSpit;
import frc.robot.commands.Commands.GroundCoralSuck;
import frc.robot.commands.Commands.WristUp;
import frc.robot.commands.Commands.WristDown;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {


  /* Controllers */
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);



  public final Servo servo1 = new Servo(0);
  
  public final Servo servo2 = new Servo(1);

  
  

  /* Subsystems */
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final AlgaeIntake AlgaeIntake = new AlgaeIntake();
  public final CoralIntake CoralIntake = new CoralIntake();
  public final Elevator Elevator = new Elevator();
  public final CoralArm CoralArm = new CoralArm();
  public final GroundCoral groundCoral = new GroundCoral();
  public final Wrist Wrist = new Wrist();

  /* Auto */
  private final SendableChooser<Command> chooser;
  private final SendableChooser<String> redBlue;
  public Pose2d startingPose;
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_robotDrive.zeroHeading();

    Map<String,Command> namedCommands = new HashMap<String,Command>();
    namedCommands.put("print test", new InstantCommand(() -> {System.out.println("Test Auto Works");}));
    NamedCommands.registerCommands(namedCommands);
    redBlue = new SendableChooser<String>();
    redBlue.addOption("red", "red");
    redBlue.addOption("blue", "blue");
    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto:", chooser);
    SmartDashboard.putData("Side:", redBlue);


    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driver.getLeftY(), OIConstants.kDriveDeadband), //left stick 
                -MathUtil.applyDeadband(driver.getLeftX(), OIConstants.kDriveDeadband), //left stick 
                -MathUtil.applyDeadband(driver.getRightX(), OIConstants.kDriveDeadband), //right stick
                true),
            m_robotDrive)
            );
           
  }        

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

  
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

     operator.rightBumper().whileTrue(new AlgaeIntakeSuck(AlgaeIntake)); 
     operator.leftBumper().whileTrue(new AlgaeIntakeSpit(AlgaeIntake));  
           
    //driver.y().toggleOnTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)); //Change this slightly to reset gyro when driving

    operator.leftTrigger().whileTrue(new CoralIntakeSuck(CoralIntake)); 
    operator.rightTrigger().whileTrue(new CoralIntakeSpit(CoralIntake)); 

    operator.leftStick().whileTrue(new ElevatorUp(Elevator)); 
    operator.leftStick().whileTrue(new ElevatorDown(Elevator)); 

    driver.a().whileTrue(new GroundCoralSpit(groundCoral)); 
    driver.b().whileTrue(new GroundCoralSuck(groundCoral)); 

    driver.x().whileTrue(new CoralArmUp(CoralArm)); 
    driver.y().whileTrue(new CoralArmDown(CoralArm)); 

    operator.rightStick().whileTrue(new WristUp(Wrist)); 
    operator.rightStick().whileTrue(new WristDown(Wrist));

    driver.povDown().whileTrue(new InstantCommand(() -> {servo1.set(0.140); System.out.println(servo1.getPosition());}));
    driver.povUp().whileTrue(new InstantCommand(() -> {servo1.set(0.45); System.out.println(servo1.getPosition());}));
    driver.povDown().whileTrue(new InstantCommand(() -> {servo2.set(0.705); System.out.println(servo2.getPosition());}));
    driver.povUp().whileTrue(new InstantCommand(() -> {servo2.set(0.385); System.out.println(servo2.getPosition());}));
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /* 
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    */

    //This will have to be changed to allow us to choose the path in the smart dashboard using a sendable chooser with autobuilder
    //return new PathPlannerAuto("testauto");
 

    
    return new SequentialCommandGroup(
      new InstantCommand(() -> { m_robotDrive.zeroHeading(); m_robotDrive.resetOdometry(startingPose); m_robotDrive.setSide(redBlue.getSelected());}),
      chooser.getSelected()
    );

    
  }
}
