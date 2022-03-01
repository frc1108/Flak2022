// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final AddressableLED m_led = new AddressableLED(LEDConstants.kLEDPort);
  private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(LEDConstants.kLEDLength); 
  
  public LEDSubsystem() {
    //m_led.setLength(LEDConstants.kLEDLength);
    m_led.setLength(m_buffer.getLength());
    m_led.setData(m_buffer);
    m_led.start();
    //setRed();
    this.setColor(255, 100, 0);

  }
  public void setRed() {
    for (var i = 0; i < m_buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_buffer.setRGB(i, 255, 0, 0);
   }
   m_led.setData(m_buffer);
  }

  public void setColor(int red, int green, int blue) {
    for (var i = 0; i < m_buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for a custom color
      m_buffer.setRGB(i, red, green, blue);
   }
   m_led.setData(m_buffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
