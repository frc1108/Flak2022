// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ClimberSubsystem extends SubsystemBase implements Loggable {
  private final CANSparkMax m_climber = new CANSparkMax(ClimberConstants.kClimberPort, MotorType.kBrushless);
  private final DoubleSolenoid m_climberTilt = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, ClimberConstants.kClimberExtendChannel, ClimberConstants.kClimberRetractChannel);
  private final RelativeEncoder m_encoder;
  private final SparkMaxLimitSwitch m_reverseLimit;
  private final SparkMaxLimitSwitch m_forwardLimit;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    stopClimber();
    m_climberTilt.set(Value.kForward);
    
    m_climber.restoreFactoryDefaults();
    m_climber.setIdleMode(IdleMode.kBrake);
    m_climber.setInverted(false);
    m_climber.setSmartCurrentLimit(40, 60);
    
    m_encoder = m_climber.getEncoder();

    m_reverseLimit = m_climber.getReverseLimitSwitch(Type.kNormallyOpen);
    m_forwardLimit = m_climber.getForwardLimitSwitch(Type.kNormallyOpen);

    m_climber.burnFlash();
    
    resetEncoder();

    // Sendable objects
    // SmartDashboard.putData("Climber Tilt", m_climberTilt);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    if(getReverseLimit()) {
      resetEncoder();
    }
  }

  public boolean getReverseLimit() {
    return m_reverseLimit.isPressed();
  }
  public boolean getForwardLimit() {
    return m_forwardLimit.isPressed();
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

  public void climber(double speed) {
    double position = m_encoder.getPosition();
    double modifier = 1;
    if (position <= 30 && position > 10){
      modifier = 0.65;
    } else if (position <= 10) {
      modifier = 0.35;
    } else if (position >=240) {
      modifier = 0.5;
    } else {
      modifier = 1;
    }
    m_climber.setVoltage(12*speed*modifier*ClimberConstants.kClimberModifier);
  }

  @Log
  public double getPosition() {
    return m_encoder.getPosition();
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }
}
