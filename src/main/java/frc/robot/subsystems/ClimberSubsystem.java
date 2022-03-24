// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_climber = new CANSparkMax(ClimberConstants.kClimberPort, MotorType.kBrushless);
  private final DoubleSolenoid m_climberTilt = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, ClimberConstants.kClimberExtendChannel, ClimberConstants.kClimberRetractChannel);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    stopClimber();
    m_climberTilt.set(Value.kReverse);

    m_climber.restoreFactoryDefaults();

    m_climber.setIdleMode(IdleMode.kBrake);

    m_climber.setInverted(false);

    m_climber.setSmartCurrentLimit(40, 60);

    m_climber.burnFlash();
  }
  
  public void toggleTilt() {
    m_climberTilt.toggle();
  }
  public void extend() {
    m_climberTilt.set(Value.kForward);
  }
  public void retract() {
    m_climberTilt.set(Value.kReverse);
  }
  
  public void stopClimber() {
    m_climber.stopMotor();
  }

  /* public void intake(boolean inverted) {
    int direction = inverted ? 1 : -1;
    m_intake.setVoltage(IntakeConstants.kIntakeVolts*direction);

  } */
  public void climber() {
    m_climber.setVoltage(ClimberConstants.kClimberSpeed);
  }
  public void climber(double modifier) {
    m_climber.setVoltage(ClimberConstants.kClimberSpeed*modifier);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
