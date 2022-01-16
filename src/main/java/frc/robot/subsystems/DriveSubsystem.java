// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftMain = new CANSparkMax(DriveConstants.kLeftMainPort, MotorType.kBrushless);
  private final CANSparkMax m_leftFollow = new CANSparkMax(DriveConstants.kLeftFollowPort, MotorType.kBrushless);
  private final CANSparkMax m_rightMain = new CANSparkMax(DriveConstants.kRightMainPort, MotorType.kBrushless);
  private final CANSparkMax m_rightFollow = new CANSparkMax(DriveConstants.kRightFollowPort, MotorType.kBrushless);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMain,m_rightMain);

  public final double m_slewSpeed = 5; //3*DriveConstants.kMaxSpeedMetersPerSecond;  // in units/s
  public final double m_slewTurn = 5; //3*DriveConstants.kMaxSpeedMetersPerSecond;
  private final SlewRateLimiter m_speedSlew = new SlewRateLimiter(m_slewSpeed);
  private final SlewRateLimiter m_turnSlew = new SlewRateLimiter(m_slewTurn);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    // Stops drive motors
    stop();

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
    m_leftMain.setSmartCurrentLimit(40, 60);
    m_leftFollow.setSmartCurrentLimit(40, 60);
    m_rightMain.setSmartCurrentLimit(40, 60);
    m_rightFollow.setSmartCurrentLimit(40, 60);

    // Burn settings into Spark MAX flash
    m_leftMain.burnFlash();
    m_leftFollow.burnFlash();
    m_rightMain.burnFlash();
    m_rightFollow.burnFlash();

    // Set drive deadband and safety 
    m_drive.setDeadband(0.05);
    m_drive.setSafetyEnabled(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(m_speedSlew.calculate(-fwd), 0.8*m_turnSlew.calculate(rot));
  }

  /**
   * Stops all the Drive subsytem motors
   */
  public void stop(){
    m_leftMain.stopMotor();
    m_leftFollow.stopMotor();
    m_rightMain.stopMotor();
    m_rightFollow.stopMotor();
  }  

    /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
}
