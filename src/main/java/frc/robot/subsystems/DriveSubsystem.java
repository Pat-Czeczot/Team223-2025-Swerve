package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.SwerveUtils;
import pabeles.concurrency.ConcurrencyOps.NewInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId, 
      DriveConstants.kRearLeftTurningCanId, 
      DriveConstants.kBackLeftChassisAngularOffset); 

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule( 
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId, 
      DriveConstants.kBackRightChassisAngularOffset);

  private AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI); // NAVX IS CW POSITIVE AND CCW NEGATIVE -- IF ISSUES WITH DRIVING OR AUTOS TRY TO NEGATE AND NORMALIZE THE ANGLE FROM .getAngle()

  // Slew rate filter variables for tuning lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  private Pose2d resetPosition = new Pose2d(new Translation2d(0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
  private double angleSetpoint = 0.0;
  private PIDController anglePIDController = new PIDController(1.0/60.0, 0, 0);

  // Odometry class for tracking the robot's pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  private boolean isPathFlipped = false;

  private SysIdRoutine r1 = new SysIdRoutine(new Config(), 
    new Mechanism(null, null, this)
  );

  // Create a new DriveSubsystem
  public DriveSubsystem() {
    anglePIDController.enableContinuousInput(-180, 180);
    m_gyro.reset();

    
    //Autobuilder || if it doesnt work, try replacing setChassisSpeeds with the drive function, call dan bc it might be difficult to implement
    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> setChassisSpeeds(speeds), //we can change this to use feedforward information but it is not necessary 
      new PPHolonomicDriveController( // YOU WILL HAVE TO TUNE THESE VALUES ALONG WITH THE OTHER PID VALUES USING SYSID for 2025, THESE ARE ONLY TEMP DEFAULTS
        new PIDConstants(0.027, 0.001, 0.001), // Translation PID constants
        new PIDConstants(0.05, 0.0053, 0.001) // Rotation PID constants
      ),
      new RobotConfig( //YOU WILL HAVE TO CALCULATE ALL OF THESE VARIABLES WHENEVER THE ROBOT SIGNIFICANTLY CHANGES, THESE ARE ONLY TEMP DEFAULTS
        52.0, //mass, kg
        4.0, //moment of intertia, kg*m^2
        new ModuleConfig(//CALCULATE THESE TOO
        0.0381, //wheel radius, meters (assuming our wheels have a 3 in diameter, double check for yourself)
        5.74, //max speed drive motor can reach when driving the robot, m/s
        1.0, //coefficient of friction between wheel and carpet, use 1 as a placeholder if have trouble calculating it
        new DCMotor(//THESE VALUES SHOULD BE CORRECT AS THEY ARE FROM THE SPECIFICATIONS OF EACH DRIVE MOTOR, the REV VORTEX
          12.0, //standard running voltage
          3.6, // stall torque, N*m
          211.0, // current draw when stalled, amps
          3.6, //current draw under no load, amps
          710.418, //angular speed when under no load, rad/s
          1 //num motors in gearbox (we only have one drive motor per module bc swerve)
        ),
        4.71, //gear reduction ratio (depends on what gear is used in the module so idk ask someone) should be either 5.50, 5.08, or 4.71 i believe
        80.0, //current limit of the drive motor, amps, this seems way too high for the robot but its what i found on rev but ask around
        1 //num of drive motors, we only have one bc swerve
        ), 
        new Translation2d(-0.29845, 0.36830), //offset of each module from center of robot in (x meters component,  y meters component) KEEP THE SIGNS, please calculate this im guessing baced on the 25 x 25in frame, front left
        new Translation2d(0.29845, 0.36830), //front right
        new Translation2d(-0.29845, -0.36830), //rear left
        new Translation2d(0.29845, -0.36830) //rear right
      ),
      this::flipPathToRedSide, //CHANGE THIS FUNCTION AS SPECIFIED BY ITS FUNCTION DEFINITON
      this
      );
  }

  //IMPORTANT!!!!! CHANGE THIS FUNCTION LATER ON TO ACCEPT INPUT FROM SMARTDASHBORD AND RETURN TRUE IF YOU ARE ON RED SIDE AND RETURN FALSE IF YOU ARE ON BLUE SIDE
  public boolean flipPathToRedSide(){ 
    return isPathFlipped;
  }

  public void setSide(String redBlue){
    if(redBlue.equals("red")){
      isPathFlipped = true;
    }else{
      isPathFlipped = false;
    }
    
  }

  // Return the currently-estimated pose of the robot
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Reset the odometry to the specified pose
  public void resetOdometry(Pose2d pose) {
    /*
    m_gyro.reset(); //remove if isnt working or try m_gyro.resetDisplacement(), that might work as well
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
    */
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }
  
  public ChassisSpeeds getRobotRelativeSpeeds(){
    ChassisSpeeds chassisSpeeds = Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
    return chassisSpeeds;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] desiredStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
    

  // Update odometry in the periodic block
  @Override
  public void periodic() {
    m_odometry.update(
      Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }


/*
public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    
      SwerveModuleState[] swerveModuleStates =
        Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, Rotation2d.fromDegrees(getHeading()))
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
      

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
*/
  
/* 
public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

  double xSpeedCommanded;
  double ySpeedCommanded;

  if (rateLimit) {
    // Convert XY to polar for rate limiting
    double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
    double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

    // Calculate the direction slew rate based on an estimate of the lateral acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
    }


    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
    if (angleDif < 0.45*Math.PI) {
      m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
    }
    else if (angleDif > 0.85*Math.PI) {
      if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      else {
        m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
    }
    else {
      m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(0.0);
    }
    m_prevTime = currentTime;

    xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.calculate(rot);
    

  } else {
    xSpeedCommanded = xSpeed;
    ySpeedCommanded = ySpeed;
    m_currentRotation = rot;
  }
  if (Math.abs(rot) > 0.05) {
    angleSetpoint -= Math.copySign(Math.pow(rot, 2) * 8, rot);
    if (angleSetpoint > 360) {
      angleSetpoint -= 360;
    } else if (angleSetpoint < 0) {
      angleSetpoint += 360;
    }
  }
  anglePIDController.setSetpoint(angleSetpoint - 180);


  // Convert the commanded speeds to the correct units for the drivetrain
  double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
  double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
  double rotDelivered = compensateAngle() * DriveConstants.kMaxAngularSpeed;
  double oculusYaw = getHeading();

  var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(oculusYaw))
          : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
  SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
  m_frontLeft.setDesiredState(swerveModuleStates[0]);
  m_frontRight.setDesiredState(swerveModuleStates[1]);
  m_rearLeft.setDesiredState(swerveModuleStates[2]);
  m_rearRight.setDesiredState(swerveModuleStates[3]);
}

*/
  private double compensateAngle() {
    var ret = -anglePIDController.calculate(m_gyro.getAngle() - 180);
    return ret;
  }

  // Set the wheels into an X formation to prevent movement
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  // Set the swerve module states
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  // Zero the encoders on the swerve modules
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  // Return the robot heading in degrees, between -180 and 180 degrees
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getYaw()).getDegrees();
     //getAngle returns a continuous angle, so you might have to convert it to the -180 to 180 range 
  }

  // Get the rotation rate of the robot
  public double getTurnRate() {
    return m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }


  public void zeroHeading() {
    m_gyro.zeroYaw();
  }
}