// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAim extends CommandBase {
  VisionSubsystem m_vision;
  DriveSubsystem m_drive;
  Boolean m_hasShooter;
  XboxController m_driverController;
  PhotonPipelineResult result;
  PIDController m_forwardPID = new PIDController(DriveConstants.kPForward, 0, 0);
  PIDController m_rotationPID = new PIDController(DriveConstants.kPTurn, 0, 0.001);

  /** Auto Aim 
   * 
   * @param drive
   * @param vision
   * @param hasShooter
   */
  public AutoAim(DriveSubsystem drive, VisionSubsystem vision, Boolean hasShooter, XboxController driverController) {
    m_drive = drive;
    m_vision = vision;
    m_hasShooter = hasShooter;
    m_driverController = driverController;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_hasShooter) {
      m_vision.lightsOn();
    }
    else {
      m_vision.lightsOff();
    }

    m_rotationPID.setSetpoint(DriveConstants.kTurnSetpoint);
    m_rotationPID.setTolerance(DriveConstants.kTurnTolerance);
    m_forwardPID.setSetpoint(DriveConstants.kForwardSetpoint);
    m_forwardPID.setTolerance(DriveConstants.kForwardTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get the latest result from the correct camera
    result = m_vision.m_reflectiveCamera.getLatestResult();
/*     if (m_hasShooter) {
      result = m_vision.m_reflectiveCamera.getLatestResult();
    }
    else {
      result = null; //m_vision.m_HD3000.getLatestResult();
    } */

    // Make sure it has a target we can use
    SmartDashboard.putBoolean("hastargets", result.hasTargets());
    if (result.hasTargets()) {
      // Get the angle of the best target needs to be inverted because Photon is + right
      double targetAngle = m_vision.getYaw(result);
      double targetDist = m_vision.distanceToShooterTarget(result);
      SmartDashboard.putNumber("Hub Angle", targetAngle);
      SmartDashboard.putNumber("Hub Dist ft", Units.metersToFeet(targetDist)+2);


      // Override the rotation input with a PID value seeking centered in the camera
      double pidAngle = -m_rotationPID.calculate(targetAngle);
      pidAngle = Math.copySign(Math.abs(pidAngle)+DriveConstants.kTurnFF, pidAngle);
      double pidDist = -m_forwardPID.calculate(targetDist);
      pidDist = Math.copySign(Math.abs(pidDist)+DriveConstants.kForwardFF,pidDist);
      SmartDashboard.putNumber("Angle Output", pidAngle);
      SmartDashboard.putNumber("Dist Output", pidDist);

      //input.m_rotation = Math.copySign(DRIVE.AIMFF + Math.abs(pidAngle), pidAngle);
      
      // If aiming at a ball we want to use robot relative movement
      if (m_hasShooter) {
        m_drive.arcadeDrive(pidDist, pidAngle);
      }
      else {
        m_drive.arcadeDrive(m_driverController.getLeftY(), m_driverController.getRightX());
      }
    }
    else {
      // If no target, drive normally 
      m_drive.arcadeDrive(m_driverController.getLeftY(), m_driverController.getRightX());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vision.lightsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}