// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.pantherlib.PicoColorSensor;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ColorSubsystem extends SubsystemBase implements Loggable {
    
  private final PicoColorSensor m_pico = new PicoColorSensor();
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.16, 0.427, 0.419);
  private final Color kRedTarget = new Color(0.561, 0.114, 0.34);
  private final Color kGreenTarget = new Color(0.197, 0.22, 0.59);
  private final Color kYellowTarget = new Color(0.33, 0.113, 0.55);
  /** Creates a new ColorSubsystem. */
  public ColorSubsystem() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Log
  public boolean getRedFrontMatch(){
    if (m_pico.isSensor0Connected()) {
      Color detectedColor = m_pico.convertRawToColor(m_pico.getRawColor0());
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      return (match.color == kRedTarget);
    } else {
    return false;
    }
  }

  @Log
  public double getRed(){
    if (m_pico.isSensor0Connected()) {
      return m_pico.convertRawToColor(m_pico.getRawColor0()).red;
    }
    else {
      return -1;
    }
  }
  @Log
  public double getBlue(){
    if (m_pico.isSensor0Connected()) {
      return m_pico.convertRawToColor(m_pico.getRawColor0()).blue;
    }
    else {
      return -1;
    }
  }
  @Log
  public double getGreen(){
    if (m_pico.isSensor0Connected()) {
      return m_pico.convertRawToColor(m_pico.getRawColor0()).green;
    }
    else {
      return -1;
    }
  }
  
  @Log
  public boolean getBlueFrontMatch(){
    if (m_pico.isSensor0Connected()) {
      Color detectedColor = m_pico.convertRawToColor(m_pico.getRawColor0());
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      return (match.color == kBlueTarget);
    } else {
    return false;
    }
  }

  @Log
  public boolean getRedBackMatch(){
    if (m_pico.isSensor1Connected()) {
      Color detectedColor = m_pico.convertRawToColor(m_pico.getRawColor1());
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      return (match.color == kRedTarget);
    } else {
    return false;
    }
  }

  @Log
  public boolean getBlueBackMatch(){
    if (m_pico.isSensor1Connected()) {
      Color detectedColor = m_pico.convertRawToColor(m_pico.getRawColor1());
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      return (match.color == kBlueTarget);
    } else {
    return false;
    }
  }

  @Log
  public boolean isFrontConnected(){
    return m_pico.isSensor0Connected();
  }

  @Log
  public boolean isBackConnected(){
    return m_pico.isSensor1Connected();
  }
}
