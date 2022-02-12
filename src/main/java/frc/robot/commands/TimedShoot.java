package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
public class TimedShoot extends SequentialCommandGroup{
    public TimedShoot(ShooterSubsystem m_shooter, double percent, double duration) {
        addCommands(
            new RunCommand(() -> {
                m_shooter.shoot(percent);
            }).withTimeout(duration)
            .andThen(
                new InstantCommand(() -> {
                    m_shooter.stopShoot();
                }
            ))
        );
    }
}
