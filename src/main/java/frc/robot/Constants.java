// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // CAN IDs for Spark Max drivetrain controllers
    public static final int kLeftMainPort = 1;
    public static final int kLeftFollowPort = 3;
    public static final int kRightMainPort = 2;
    public static final int kRightFollowPort = 4;

    // Physical robot parameters
    public static final int kEncoderCPR = 42;  // NEO motor encoder Counts per revolution
    public static final double kGearRatio = 8.45; // Toughbox mini gear ratio
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6); // Wheel diameter
    // Use sysid angular to determine the best robot wheel width (may not match tape measurer)
    public static final double kTrackwidthMeters = Units.inchesToMeters(22);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final Boolean kGyroReversed = false; 
    // WPIlib is ccw positive, use to invert gyro match
    // NavX = true, ADIS16470 = false

    // Encoder count conversion on the spark max for NEOs from rotations to SI units 
    public static final double kEncoderDistanceConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(kGearRatio));
    public static final double kEncoderVelocityConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(60*kGearRatio));

    // Tuning parameters, use sysid to determine values
    public static final double ksVolts = 0.17387;   //0.169
    public static final double kvVoltSecondsPerMeter = 2.3243;  //2.24
    public static final double kaVoltSecondsSquaredPerMeter = 0.39228;  //0.0435
    public static final double kPDriveVel = 2.5205;  //2.4 8/14 2.24 Tuning to get better PIDF response

    // Log of sysID values
    // Feb12 Drivebase Ks = 0.17387 Kv = 2.3243 Ka = 0.39228 Kp = 2.5205
    // Jan16 Drivebase no weight Ks = 0.19627 Kv = 2.7762 Ka = 0.14895 Kp = 2.6295

    // PID turning parameter and closed loop driving paramters. Uncomment to use
    // public static final double kTurnP = 0.0475; //0.94, 0.125
    // public static final double kTurnI = 0.00;
    // public static final double kTurnD = 0.00175; //0.04, 0.0085
    // public static final double kMinCommand = 0.0;
    // public static final double kMaxTurnRateDegPerS = 360;
    // public static final double kMaxTurnAccelerationDegPerSSquared = 480;
    // public static final double kTurnToleranceDeg = 4; //0.5
    // public static final double kTurnRateToleranceDegPerS = 8;
    // public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(8);
  }
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kOperatorLeftDeadband = 0.1; //the range around zero
    public static final double kDriverSlowModifier = 0.4; //modifier for slow mode
  }
  public static final class ShooterConstants {
    public static final double kShooterPercent = 38.5;
    public static final int kLeftShooterPort = 20;
    public static final int kRightShooterPort = 21;
    public static final int kKickInPort = 41;

    public static final int kPlateDownChannel = 2;
    public static final int kPlateUpChannel = 3;

    public static final int kTiltExtendChannel = 1;
    public static final int kTiltRetractChannel = 0;
  }
  public static final class IntakeConstants {
    public static final int kIntakePort = 42;
    public static final int kIntakeVolts = 6;
    public static final double kIntakeModifier = 8.5; //12 is the max

    public static final int kIntakeExtendChannel = 5;
    public static final int kIntakeRetractChannel = 4;
  }
  public static final class AutoConstants {
    // Adjust the max robot speed during auto trajectories ~80% of max speed
    // During testing set these values lower to check for correct movements
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(8);
    public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(8);

    // Tuning parameter for RAMSETE that don't need changed
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    
    // Constraints for trajectory to reduce speed around tight corners
    public static final double kmaxCentripetalAccelerationMetersPerSecondSq = 0.03;
    public static final double  kDifferentialDriveKinematicsConstraint = 0.3;
  }
  public static final class LEDConstants {
    public static final int kLEDPort = 9;
    public static final int kLEDLength = 150;
  }
}