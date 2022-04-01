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

public class FourBallShort extends SequentialCommandGroup {
  public FourBallShort(DriveSubsystem m_robotDrive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {        
      TrajectoryConfig fwdConfig = new TrajectoryConfig(3, 3);
      TrajectoryConfig fwdConfigSlow = new TrajectoryConfig(2.5, 2.5);
      TrajectoryConfig revConfig = new TrajectoryConfig(3, 3).setReversed(true);
      TrajectoryConfig revConfigSlow = new TrajectoryConfig(2.5, 2.5).setReversed(true);
      
      Trajectory trajToCargo = m_robotDrive.generateTrajectory("ShortCargo1", fwdConfigSlow);
      Trajectory reverseTurn = m_robotDrive.generateTrajectory("Cargo2", revConfigSlow);
      Trajectory terminalRun = m_robotDrive.generateTrajectory("Cargo3", fwdConfig);
      Trajectory returnToHub = m_robotDrive.generateTrajectory("Cargo4", revConfig);
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajToCargo.getInitialPose());
          }),
          parallel(
            m_robotDrive.createCommandForTrajectory(trajToCargo, false).withTimeout(5).withName("Cargo One Pickup"),
            new InstantCommand(()->m_intake.extend()),
            new TimedIntake(m_intake, 2.7),
            new TimedKick(m_shooter, 2.7)),
          new InstantCommand(()->m_shooter.tiltUp()),
          //new TimedKick(m_shooter, 1),
          new Shoot(m_shooter),
          parallel(
            new InstantCommand(()->m_shooter.tiltDown()),
            m_robotDrive.createCommandForTrajectory(reverseTurn, false).withTimeout(5).withName("Reverse Turn")),
          deadline(
            m_robotDrive.createCommandForTrajectory(terminalRun, false).withTimeout(5).withName("Terminal Run").andThen(new WaitCommand(0.55)),
            new TimedKick(m_shooter,10), //this time is long enough that the stop command doesnt get scheduled because of the deadline group
            sequence(
              new TimedIntake(m_intake, 2),
                new TimedIntake(m_intake, 0.075, -.25),
                new TimedIntake(m_intake, 0.3),
                new TimedIntake(m_intake, 0.075, -.25),
                new TimedIntake(m_intake, 10)/* ,
                new TimedIntake(m_intake, 0.075, -.25),
                new TimedIntake(m_intake, 0.3),
                new TimedIntake(m_intake, 0.075, -.25),
                new TimedIntake(m_intake, 10) */)), 
          parallel(
            m_robotDrive.createCommandForTrajectory(returnToHub, false).withTimeout(5).withName("Return To Hub"),
            sequence(
              new WaitCommand(1.25),
              new InstantCommand(()->m_shooter.tiltUp()),
              new WaitCommand(0.25),
              //new InstantCommand(()->m_intake.stopIntake()),
              new WaitCommand(1),
              new InstantCommand(()->m_shooter.stopKick()) 
              ),
            deadline(
              new WaitCommand(1.5),
              sequence(
                new TimedIntake(m_intake, 0.3),
                new TimedIntake(m_intake, 0.075, -.25),
                new TimedIntake(m_intake, 0.3),
                new TimedIntake(m_intake, 0.075, -.25),
                new TimedIntake(m_intake, 0.3),
                new TimedIntake(m_intake, 0.075, -.25),
                new TimedIntake(m_intake, 0.3),
                new TimedIntake(m_intake, 0.075, -.25),
                new TimedIntake(m_intake, 0.3),
                new TimedIntake(m_intake, 0.075, -.25),
                new TimedIntake(m_intake, 0.3),
                new TimedIntake(m_intake, 0.075, -.25)
              ))/* .andThen(new InstantCommand(()->m_intake.stopIntake())) */),
          new Shoot(m_shooter),
          new InstantCommand(()->m_intake.stopIntake()),
          new InstantCommand(()->m_shooter.tiltDown())
      );
  }
}
