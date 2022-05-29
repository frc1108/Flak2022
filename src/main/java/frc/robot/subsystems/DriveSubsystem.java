// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.pantherlib.Trajectory6391;
import io.github.oblarg.oblog.Loggable;

import java.io.IOException;
import java.nio.file.Paths;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public class DriveSubsystem extends SubsystemBase implements Loggable {
  private final CANSparkMax m_leftMain = new CANSparkMax(DriveConstants.kLeftMainPort, MotorType.kBrushless);
  private final CANSparkMax m_leftFollow = new CANSparkMax(DriveConstants.kLeftFollowPort, MotorType.kBrushless);
  private final CANSparkMax m_rightMain = new CANSparkMax(DriveConstants.kRightMainPort, MotorType.kBrushless);
  private final CANSparkMax m_rightFollow = new CANSparkMax(DriveConstants.kRightFollowPort, MotorType.kBrushless);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMain,m_rightMain);

  public final double m_slewSpeed = 4.25; // in units/s
  public final double m_slewTurn = 4.25;
  private final SlewRateLimiter m_speedSlew = new SlewRateLimiter(m_slewSpeed);
  private final SlewRateLimiter m_turnSlew = new SlewRateLimiter(m_slewTurn);

  private final RelativeEncoder m_leftEncoder, m_rightEncoder;
  private final DifferentialDriveOdometry m_odometry;
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final ADIS16470_IMUSim m_gyroSim;

  // These classes help us simulate our drivetrain
  public DifferentialDrivetrainSim m_drivetrainSimulator;
  
  // The Field2d class shows the field in the sim GUI
  private final Field2d m_fieldSim;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    // Stops drive motors
    idle();

    // Restores default CANSparkMax settings
    m_leftMain.restoreFactoryDefaults();
    m_leftFollow.restoreFactoryDefaults();
    m_rightMain.restoreFactoryDefaults();
    m_rightFollow.restoreFactoryDefaults();

    m_leftFollow.follow(m_leftMain);
    m_rightFollow.follow(m_rightMain);

    m_leftMain.setInverted(false);
    m_rightMain.setInverted(true);
    
    // Set Idle mode for CANSparkMax (brake)
    m_leftMain.setIdleMode(IdleMode.kBrake);
    m_leftFollow.setIdleMode(IdleMode.kBrake);
    m_rightMain.setIdleMode(IdleMode.kBrake);
    m_rightFollow.setIdleMode(IdleMode.kBrake);
    
    // Set Smart Current Limit for CAN SparkMax
    // Higher Limit at Stall than at Free Speed
    m_leftMain.setSmartCurrentLimit(46, 40);
    m_leftFollow.setSmartCurrentLimit(46, 40);
    m_rightMain.setSmartCurrentLimit(46, 40);
    m_rightFollow.setSmartCurrentLimit(46, 40);

    // Setup NEO internal encoder to return SI units for odometry
    m_leftEncoder = m_leftMain.getEncoder();
    m_rightEncoder = m_rightMain.getEncoder();
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_rightEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_leftEncoder.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);

    // Burn settings into Spark MAX flash
    m_leftMain.burnFlash();
    m_leftFollow.burnFlash();
    m_rightMain.burnFlash();
    m_rightFollow.burnFlash();

    // Set drive deadband and safety 
    m_drive.setDeadband(0.05);
    m_drive.setSafetyEnabled(true);

    if (RobotBase.isSimulation()) { //If our robot is simulated
      m_drivetrainSimulator = 
      new DifferentialDrivetrainSim(
        DriveConstants.kDrivetrainPlant,
        DriveConstants.kDriveGearbox,
        DriveConstants.kDriveGearing,
        DriveConstants.kTrackwidthMeters,
        DriveConstants.kWheelDiameterMeters / 2.0,
          VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

          m_gyroSim = new ADIS16470_IMUSim(m_gyro);
    } else {
      m_gyroSim = null;
    }

    m_fieldSim = new Field2d();

    // Start robot odometry tracker
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    SmartDashboard.putData("Differential Drive", m_drive);
    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putData("Field", m_fieldSim);
  }

  /**
   * Updating values to logging and odometry, or other periodic updates
   */
  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Dist", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Dist", m_rightEncoder.getPosition());
    
    // Place our robot on the field based on our odometry
    m_fieldSim.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {

    // Set the inputs to the drivesim based on the current robot voltage
    // Note: The motor controllers give us a negative output value because they are inverted, so we have to invert that value.
    m_drivetrainSimulator.setInputs(
      -m_leftMain.get()*RobotController.getInputVoltage(),
      -m_rightMain.get()*RobotController.getInputVoltage());
    m_drivetrainSimulator.update(0.020);

    // Update the Spark Max positions
    setSimDoubleFromDeviceData("SPARK MAX [1]", "Position", m_drivetrainSimulator.getLeftPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [3]", "Position", m_drivetrainSimulator.getLeftPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [2]", "Position", m_drivetrainSimulator.getRightPositionMeters());
    setSimDoubleFromDeviceData("SPARK MAX [4]", "Position", m_drivetrainSimulator.getRightPositionMeters());

    // Update the Spark Max applied outputs
    setSimDoubleFromDeviceData("SPARK MAX [1]", "Applied Output", m_leftMain.get());
    setSimDoubleFromDeviceData("SPARK MAX [3]", "Applied Output", m_leftMain.get());
    setSimDoubleFromDeviceData("SPARK MAX [2]", "Applied Output", m_rightMain.get());
    setSimDoubleFromDeviceData("SPARK MAX [4]", "Applied Output", m_rightMain.get());
    m_gyroSim.setGyroAngleX(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /**
   * Modify the JSON of the specified object to set the SIM Double
   * @param deviceName
   * @param doubleName
   * @param value
   */
  public void setSimDoubleFromDeviceData(String deviceName, String doubleName, double value) {
    int device = SimDeviceDataJNI.getSimDeviceHandle(deviceName);
    SimDouble simDouble = new SimDouble(SimDeviceDataJNI.getSimValueHandle(device, doubleName));
    simDouble.set(value);
  }

    /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
   * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return m_drivetrainSimulator.getCurrentDrawAmps();
  }

  /***** Drivetrain methods
   * stop: set motors to zero
   * setMaxOutput: set drivetrain max speed
   * arcadeDrive: set output of drive motors with robot speed and rotation
   * tankDriveVolts: set voltage of drive motors directly
   * tankDriveFeedforwardPID: set wheel speed of drive motors closed loop
   */

  public void idle(){
    m_leftMain.stopMotor();
    m_leftFollow.stopMotor();
    m_rightMain.stopMotor();
    m_rightFollow.stopMotor();
  }

  public void changeIdleMode(IdleMode idleMode) {
    m_leftMain.setIdleMode(idleMode);
    m_leftFollow.setIdleMode(idleMode);
    m_rightMain.setIdleMode(idleMode);
    m_rightFollow.setIdleMode(idleMode);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(m_speedSlew.calculate(-fwd), 0.8*m_turnSlew.calculate(rot));
  }

  public void curvatureDrive(double fwd, double rot, boolean quickTurn) {
    m_drive.curvatureDrive(m_speedSlew.calculate(-fwd), m_turnSlew.calculate(rot), quickTurn);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMain.setVoltage(leftVolts);
    m_rightMain.setVoltage(rightVolts);
    m_drive.feed();
}
  
  /***** Gyro methods
   * zeroHeading: sets gyro to zero
   * getHeading: returns gyro angle in degrees
   * getHeadingCW: returns gyro angle in degrees clockwise
   * getTurnRateCW: returns gyro rate in degrees/sec clockwise
   */

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360)*(DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getHeadingCW() {
    return Math.IEEEremainder(-m_gyro.getAngle(), 360);
  }

  public double getTurnRateCW() {
    return -m_gyro.getRate();
  }

   /***** Encoder methods
   * resetEncoders: sets encoders to zero
   * getAverageEncoderDistance: get combined left and right changes in position
   * getLeftEncoder: get left RelativeEncoder
   * getRightEncoder: get right RelativeEncoder
   */

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /***** Odometry methods - keep track of robot pose 
   * resetOdometry: Set odometry position with reset encoder and gyro
   * getPose: Get current robot pose
   * getWheelSpeeds: 
  */

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    zeroHeading();
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(),m_rightEncoder.getVelocity());
  } 

  /***** Trajectory methods - make paths for robot to follow 
   * createCommandForTrajectory:
   * generateTrajectory:
   * generateTrajectoryFromFile:
   */

  public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {
    if (initPose) {
      new InstantCommand(() -> {resetOdometry(trajectory.getInitialPose());});
    }

    resetEncoders();

    RamseteCommand ramseteCommand =  new RamseteCommand(trajectory, this::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics, this::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0), this::tankDriveVolts, this);
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
  }

  public Trajectory generateTrajectory(String trajectoryName, TrajectoryConfig config) {
    try {
      var filepath = Filesystem.getDeployDirectory().toPath().resolve(Paths.get("waypoints", trajectoryName));
      return Trajectory6391.fromWaypoints(filepath, config);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + trajectoryName, false);
      return new Trajectory();
    }
  }

  public Trajectory generateTrajectoryFromFile(String filename) {
      var config = new TrajectoryConfig(1, 3);
      return generateTrajectory(filename, config);
  }
}