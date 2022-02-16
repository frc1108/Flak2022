package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootOnce;
import frc.robot.commands.TimedIntake;
import frc.robot.commands.TimedKick;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FourBallAuto extends SequentialCommandGroup {
  public FourBallAuto(DriveSubsystem m_robotDrive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {        
      TrajectoryConfig fwdConfig = new TrajectoryConfig(2, 3);
      TrajectoryConfig revConfig = new TrajectoryConfig(2, 3).setReversed(true);
      
      Trajectory trajToCargo = m_robotDrive.generateTrajectory("Cargo1", fwdConfig);
      Trajectory reverseTurn = m_robotDrive.generateTrajectory("Cargo2", revConfig);
      Trajectory terminalRun = m_robotDrive.generateTrajectory("Cargo3", fwdConfig);
      Trajectory returnToHub = m_robotDrive.generateTrajectory("Cargo4", revConfig);
      
      addCommands(
          new ShootOnce(m_shooter),
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajToCargo.getInitialPose());
          }),
          deadline(
            m_robotDrive.createCommandForTrajectory(trajToCargo, false).withTimeout(5).withName("Cargo One Pickup"),
            new InstantCommand(()-> {
                m_intake.extend();
                }),
            new TimedIntake(m_intake, 3),
            new TimedKick(m_shooter, 3)),
          new ShootOnce(m_shooter),
          m_robotDrive.createCommandForTrajectory(reverseTurn, false).withTimeout(5).withName("Reverse Turn")/* ,
          m_robotDrive.createCommandForTrajectory(terminalRun, false).withTimeout(5).withName("Terminal Run"),
          m_robotDrive.createCommandForTrajectory(returnToHub, false).withTimeout(5).withName("Return To Hub") */
      );
  }
}
