// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.IntakeConstants;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeSubsystem extends SubsystemBase implements Loggable {
  private final CANSparkMax m_intake = new CANSparkMax(IntakeConstants.kIntakePort, MotorType.kBrushless);
  private final DoubleSolenoid m_intakeExtender = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, IntakeConstants.kIntakeExtendChannel, IntakeConstants.kIntakeRetractChannel);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    stopIntake();
    m_intakeExtender.set(Value.kReverse);

    m_intake.restoreFactoryDefaults();

    m_intake.setIdleMode(IdleMode.kBrake);

    m_intake.setInverted(false);

    m_intake.setSmartCurrentLimit(40, 60);

    m_intake.burnFlash();
  }
  
  public void toggleExtension() {
    m_intakeExtender.toggle();
  }
  public void extend() {
    m_intakeExtender.set(Value.kForward);
  }
  public void retract() {
    m_intakeExtender.set(Value.kReverse);
  }

  public void stopIntake() {
    m_intake.stopMotor();
  }

  /* public void intake(boolean inverted) {
    int direction = inverted ? 1 : -1;
    m_intake.setVoltage(IntakeConstants.kIntakeVolts*direction);

  } */
  public void intake(double speed) {
    m_intake.setVoltage(speed*IntakeConstants.kIntakeModifier);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
