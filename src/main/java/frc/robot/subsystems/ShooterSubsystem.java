// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants.ShooterConstants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftShooter = new CANSparkMax(ShooterConstants.kLeftShooterPort, MotorType.kBrushless);
  private final CANSparkMax m_rightShooter = new CANSparkMax(ShooterConstants.kRightShooterPort, MotorType.kBrushless);
  private final CANSparkMax m_kickIn = new CANSparkMax(ShooterConstants.kKickInPort, MotorType.kBrushed);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    stopKick();
    stopShoot();
    m_leftShooter.restoreFactoryDefaults();
    m_rightShooter.restoreFactoryDefaults();
    m_kickIn.restoreFactoryDefaults();

    m_rightShooter.setIdleMode(IdleMode.kCoast);
    m_leftShooter.setIdleMode(IdleMode.kCoast);
    m_kickIn.setIdleMode(IdleMode.kBrake);

    m_leftShooter.setInverted(false);
    m_kickIn.setInverted(false);
    //this makes the right shooter follow the left, but inversed = true
    m_rightShooter.follow(m_leftShooter, true);

    m_rightShooter.setSmartCurrentLimit(40, 60);
    m_leftShooter.setSmartCurrentLimit(40, 60);

    m_rightShooter.burnFlash();
    m_leftShooter.burnFlash();
    m_kickIn.burnFlash();
  }
  /* public void stopAll() {
    m_rightShooter.stopMotor();
    m_kickIn.stopMotor();
  } */
  public void stopKick() {
    m_kickIn.stopMotor();
  }
  public void stopShoot() {
    m_leftShooter.stopMotor();
  }
  private static double convertPercentTo12Volts (double percent) {
    return percent*(3/25);
  }

  /**
   * Controls the shooting Neo motors.
   * @param speedPercent Speed on a scale from 0 to 100
   */
  public void shoot(double speedPercent) {
    m_leftShooter.setVoltage(convertPercentTo12Volts(speedPercent));
  }

  /**
   * Controls the kick in redline motor
   * @param speed Speed Percent from 0 to 100
   */
  public void kick(double speed) {
    m_kickIn.setVoltage(convertPercentTo12Volts(speed));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
