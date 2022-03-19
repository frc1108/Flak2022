// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Shoot;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.counter.Tachometer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShooterSubsystem extends SubsystemBase implements Loggable{
  private final CANSparkMax m_leftShooter = new CANSparkMax(ShooterConstants.kLeftShooterPort, MotorType.kBrushless);
  private final CANSparkMax m_rightShooter = new CANSparkMax(ShooterConstants.kRightShooterPort, MotorType.kBrushless);

  private final SimpleMotorFeedforward m_leftFF = new SimpleMotorFeedforward(ShooterConstants.kLeftShooterKs, ShooterConstants.kLeftShooterKv, ShooterConstants.kLeftShooterKa);
  private final SimpleMotorFeedforward m_rightFF = new SimpleMotorFeedforward(ShooterConstants.kRightShooterKs, ShooterConstants.kRightShooterKv, ShooterConstants.kRightShooterKa);
  public final SparkMaxPIDController m_pidLeftController, m_pidRightController;
  private RelativeEncoder m_leftEncoder, m_rightEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, m_shooterSetpoint, m_shooterTolerance;

  //private final Tachometer m_rightTach = new Tachometer(new DigitalInput(0));
  private final Tachometer m_leftTach = new Tachometer(new DigitalInput(0));

  private final CANSparkMax m_kickIn = new CANSparkMax(ShooterConstants.kKickInPort, MotorType.kBrushed);
  private final DoubleSolenoid m_plate = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, ShooterConstants.kPlateUpChannel, ShooterConstants.kPlateDownChannel);
  private final DoubleSolenoid m_tilt = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, ShooterConstants.kTiltExtendChannel, ShooterConstants.kTiltRetractChannel);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    stopKick();
    stopShoot();
    m_leftShooter.restoreFactoryDefaults();
    m_rightShooter.restoreFactoryDefaults();
    m_kickIn.restoreFactoryDefaults();
    m_plate.set(Value.kReverse);
    m_tilt.set(Value.kReverse);

    m_pidLeftController = m_leftShooter.getPIDController();
    m_pidRightController = m_rightShooter.getPIDController();
    m_pidLeftController.setP(ShooterConstants.kLeftShooterKp);    
    m_pidRightController.setP(ShooterConstants.kRightShooterKp); 
    m_leftEncoder = m_leftShooter.getEncoder();
    m_rightEncoder = m_rightShooter.getEncoder(); 

    m_rightShooter.setIdleMode(IdleMode.kCoast);
    m_leftShooter.setIdleMode(IdleMode.kCoast);
    m_kickIn.setIdleMode(IdleMode.kBrake);

    m_leftShooter.setInverted(false);
    m_kickIn.setInverted(false);
    //this makes the right shooter follow the left, but inversed = true
    //m_rightShooter.follow(m_leftShooter, true);
    m_rightShooter.setInverted(true);

    m_rightShooter.setSmartCurrentLimit(40, 40);
    m_leftShooter.setSmartCurrentLimit(40, 40);

    m_rightShooter.burnFlash();
    m_leftShooter.burnFlash();
    m_kickIn.burnFlash();


    this.setDefaultCommand(new RunCommand(() -> stopAll(), this));
  }

  public void togglePlate() {
    m_plate.toggle();
  }
  public void plateUp() {
    m_plate.set(Value.kForward);
  }
  public void plateDown() {
    m_plate.set(Value.kReverse);
  }

  public void toggleTilt() {
    m_tilt.toggle();
  }

  public void stopAll() {
    m_leftShooter.stopMotor();
    m_rightShooter.stopMotor();
    m_kickIn.stopMotor();
  } 
  public void stopKick() {
    m_kickIn.stopMotor();
  }
  public void stopShoot() {
    m_leftShooter.stopMotor();
    m_rightShooter.stopMotor();
  }
  private static double convertPercentTo12Volts (double percent) {
    return percent*3/25;
  }

  /**
   * Controls the shooting Neo motors.
   * @param speedPercent Speed on a scale from 0 to 100
   */
  public void shoot(double speedPercent) {
    double slower = speedPercent-15;
    m_leftShooter.setVoltage(convertPercentTo12Volts(slower));
    m_rightShooter.setVoltage(convertPercentTo12Volts(speedPercent));
  } 
  /* public void shoot(double volts) {
    m_leftShooter.setVoltage(volts);
  } */

  /**
   * Controls the kick in redline motor
   * @param speed Speed Percent from 0 to 100
   */
  public void kick(double speed) {
    m_kickIn.setVoltage(convertPercentTo12Volts(speed));
  }
  
  @Override
  public void periodic() {
  }

  @Config.NumberSlider(name = "Shooter Setpoint",
                       tabName = "Live",
                       defaultValue = 1750,
                       min = 0,
                       max = 4000,
                       blockIncrement = 100)
  public void setShooterSetpoint(double rpm) {
    m_shooterSetpoint = rpm/60; // Constant in rot/s, this allows rpm w/ max ~5800 for NEO 1:1
  }

  public void startShooter(){
    m_pidLeftController.setReference(
      m_shooterSetpoint, ControlType.kVelocity, 0, m_leftFF.calculate(m_shooterSetpoint));
    m_pidRightController.setReference(
      m_shooterSetpoint, ControlType.kVelocity, 0, m_rightFF.calculate(m_shooterSetpoint));
  }

  public double getSetpoint(){
    return m_shooterSetpoint;
  }

  @Log(name = "Left Shooter Speed",tabName = "Live")
  public double getLeftShooterRPM(){
     return m_leftEncoder.getVelocity();
  }

  @Log(name = "Right Shooter Speed",tabName = "Live")
  public double getRightShooterRPM(){
    return m_rightEncoder.getVelocity();
  }

  @Log
  public boolean atSpeed() {
    return Math.abs(m_leftEncoder.getVelocity() - m_shooterSetpoint) < m_shooterTolerance
        && Math.abs(m_rightEncoder.getVelocity() - m_shooterSetpoint) < m_shooterTolerance
        && Math.abs(m_leftEncoder.getVelocity()) > 0;
  }

  @Config.NumberSlider(name = "Shooter Tolerance rpm",tabName = "Live",defaultValue = 100)
  public void setShooterTolerance(double tol) {
    m_shooterTolerance = tol;
  }

  public void increaseSetpoint(){
    m_shooterSetpoint += 50;
  }

  public void decreaseSetpoint(){
    m_shooterSetpoint += 50;
  }

  public void resetSetpoint(){
    m_shooterSetpoint = 1750;
  }

  @Log
  public double getLeftTach(){
    return m_leftTach.getRevolutionsPerMinute();
  }
}
