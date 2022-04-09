package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.TimedIntake;
import frc.robot.commands.TimedKick;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RudeTwoBallAuto extends SequentialCommandGroup {
  public RudeTwoBallAuto(DriveSubsystem m_robotDrive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {        
      TrajectoryConfig fwdConfig = new TrajectoryConfig(3, 3);
      TrajectoryConfig revConfigS = new TrajectoryConfig(2, 2).setReversed(true);
      TrajectoryConfig revConfig = new TrajectoryConfig(3, 3).setReversed(true);
      
      Trajectory toCargo = m_robotDrive.generateTrajectory("Two1", fwdConfig);
      Trajectory backToTarmac = m_robotDrive.generateTrajectory("Two2", revConfigS);
      Trajectory rude = m_robotDrive.generateTrajectory("Rude", revConfig);
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(toCargo.getInitialPose());
          }),
          parallel(
            m_robotDrive.createCommandForTrajectory(toCargo, false).withTimeout(5).withName("Cargo One Pickup"),
            new InstantCommand(()->m_intake.extend()),
            new TimedIntake(m_intake, 2.8),
            new TimedKick(m_shooter, 2.8)),
          new InstantCommand(()->m_shooter.tiltUp()),
          m_robotDrive.createCommandForTrajectory(backToTarmac, false).withTimeout(3).withName("Back To Tarmac"),
          new WaitCommand(1),
          new Shoot(m_shooter),
          new InstantCommand(()->m_shooter.tiltDown()),
          new WaitCommand(0.5),
          m_robotDrive.createCommandForTrajectory(rude, false).withTimeout(4).withName("Rude ball Knock")
      );
  }
}