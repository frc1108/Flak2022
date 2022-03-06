package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootOnce;
import frc.robot.commands.TimedIntake;
import frc.robot.commands.TimedKick;
import frc.robot.commands.TimedShoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OneBallAuto extends SequentialCommandGroup {
 public OneBallAuto(DriveSubsystem m_robotDrive, ShooterSubsystem m_shooter, IntakeSubsystem m_intake) {        
      
      addCommands(
          new ShootOnce(m_shooter),
          new WaitCommand(1),
          new RunCommand(()->{
            m_robotDrive.arcadeDrive(-0.6, 0);
          }).withTimeout(1.65),
          new InstantCommand(()->{
            m_robotDrive.arcadeDrive(0, 0);;
          }), 
          new InstantCommand(()->m_shooter.tiltDown())
      );
  }
}

