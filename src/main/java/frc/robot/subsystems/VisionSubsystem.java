// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

  public PhotonCamera m_reflectiveCamera = new PhotonCamera("ircam");
  //public PhotonCamera m_cargoCamera = new PhotonCamera("pi zero")

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    m_reflectiveCamera.setPipelineIndex(VisionConstants.kReflectivePipeline);
    lightsOff();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void lightsOn() {
    m_reflectiveCamera.setLED(VisionLEDMode.kOn);
 }

 public void lightsOff() {
    m_reflectiveCamera.setLED(VisionLEDMode.kOff);
 }

 public double getYaw(PhotonPipelineResult result) {
    if (result.hasTargets()) {
       return result.getBestTarget().getYaw();
    }
    return -999.0;
 }

 public double distanceToShooterTarget(PhotonPipelineResult result) {
    return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kReflectiveCameraHeight,
                VisionConstants.kTargetHeight,
                VisionConstants.kReflectiveCameraPitch,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
 }
}
