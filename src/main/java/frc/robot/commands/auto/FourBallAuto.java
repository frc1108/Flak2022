package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootOnce;
import frc.robot.commands.TimedIntake;
import frc.robot.commands.TimedKick;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FourBallAuto extends SequentialCommandGroup {
  public FourBallAuto(DriveSubsystem m_robotDrive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {        
      TrajectoryConfig fwdConfig = new TrajectoryConfig(2.5, 3);
      TrajectoryConfig revConfig = new TrajectoryConfig(2.5, 3).setReversed(true);
      
      Trajectory trajToCargo = m_robotDrive.generateTrajectory("Cargo1", fwdConfig);
      Trajectory reverseTurn = m_robotDrive.generateTrajectory("Cargo2", revConfig);
      Trajectory terminalRun = m_robotDrive.generateTrajectory("Cargo3", fwdConfig);
      Trajectory returnToHub = m_robotDrive.generateTrajectory("Cargo4", revConfig);
      
      addCommands(
          new ShootOnce(m_shooter),
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajToCargo.getInitialPose());
          }),
          parallel(
            m_robotDrive.createCommandForTrajectory(trajToCargo, false).withTimeout(5).withName("Cargo One Pickup"),
            new InstantCommand(()->m_intake.extend()),
            new TimedIntake(m_intake, 3),
            new TimedKick(m_shooter, 3)),
          new InstantCommand(()->m_shooter.tiltUp()),
          new TimedKick(m_shooter, 1),
          new ShootOnce(m_shooter, 2.75),
          parallel(
            new InstantCommand(()->m_shooter.tiltDown()),
            m_robotDrive.createCommandForTrajectory(reverseTurn, false).withTimeout(5).withName("Reverse Turn")),
          parallel(
            m_robotDrive.createCommandForTrajectory(terminalRun, false).withTimeout(5).withName("Terminal Run"),
            new TimedIntake(m_intake, 5),
            new TimedKick(m_shooter, 5)),
          parallel(
            m_robotDrive.createCommandForTrajectory(returnToHub, false).withTimeout(5).withName("Return To Hub"),
            new InstantCommand(()->m_shooter.tiltUp())),
          new Shoot(m_shooter, 2.25),
          new InstantCommand(()->m_shooter.tiltDown())
      );
  }
}
