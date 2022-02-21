// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class VisionSubsystem extends SubsystemBase implements Loggable {
  PhotonCamera camera = new PhotonCamera("SnakeIR");

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Log
  public double getYaw() {
    return camera.getBestTarget();
  }

  @Log
  public boolean hasTarget() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      double range =
            PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.kSnakeHeight, VisionConstants.kTargetHeight, cameraPitchRadians, targetPitchRadians)
    }
  }
}
