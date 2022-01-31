// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants.ShooterConstants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_leftShooter = new CANSparkMax(ShooterConstants.kLeftShooterPort, MotorType.kBrushless);
  private final CANSparkMax m_rightShooter = new CANSparkMax(ShooterConstants.kRightShooterPort, MotorType.kBrushless);
  private final VictorSPX m_kickIn = new VictorSPX(ShooterConstants.kKickInPort);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    stop();
    m_leftShooter.restoreFactoryDefaults();
    m_rightShooter.restoreFactoryDefaults();
    m_kickIn.configFactoryDefault();

    m_rightShooter.setIdleMode(IdleMode.kCoast);
    m_leftShooter.setIdleMode(IdleMode.kCoast);
    m_kickIn.setNeutralMode(NeutralMode.Brake);

    m_leftShooter.setInverted(true);
    m_rightShooter.setInverted(false);

    m_leftShooter.follow(m_rightShooter);

    m_rightShooter.setSmartCurrentLimit(40, 60);
    m_leftShooter.setSmartCurrentLimit(40, 60);

    m_rightShooter.burnFlash();
    m_leftShooter.burnFlash();
  }
  public void stop() {
    m_rightShooter.stopMotor();
    m_kickIn.neutralOutput();
  }
  private static double convertPercentToVolts (double percent) {
    return percent*(3/25);
  }

  /**
   * Controls the shooting Neo motors.
   * @param speedPercent Speed on a scale from 0 to 100
   */
  public void shoot(double speedPercent) {
    m_rightShooter.setVoltage(convertPercentToVolts(speedPercent));
  }

  /**
   * Controls the kick in redline motor
   * @param speed Speed Percent from -1.0 to 1.0
   */
  public void kick(double speed) {
    m_kickIn.set(ControlMode.PercentOutput, speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
