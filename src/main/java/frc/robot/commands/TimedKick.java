package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
public class TimedKick extends SequentialCommandGroup{
    public TimedKick(ShooterSubsystem m_shooter, double seconds) {
        addCommands(
            new RunCommand(() -> {
                m_shooter.kick(75);
            }).withTimeout(seconds),
            new InstantCommand(() -> {
                m_shooter.stopKick();
            })
        );
    }
    public TimedKick(ShooterSubsystem m_shooter, double seconds, double multiplier) {
        addCommands(
            new RunCommand(() -> {
                m_shooter.kick(75*multiplier);
            }).withTimeout(seconds),
            new InstantCommand(() -> {
                m_shooter.stopKick();
            })
        );
    }
}
