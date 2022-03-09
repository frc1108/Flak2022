package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
public class TimedIntake extends SequentialCommandGroup{
    public TimedIntake(IntakeSubsystem m_intake, double seconds) {
        addCommands(
            new RunCommand(() -> {
                m_intake.intake();
            }).withTimeout(seconds),
            new InstantCommand(() -> {
                m_intake.stopIntake();
            })
        );
        addRequirements(m_intake);
    }
    public TimedIntake(IntakeSubsystem m_intake, double seconds, double modifier) {
        addCommands(
            new RunCommand(() -> {
                m_intake.intake(modifier);
            }).withTimeout(seconds),
            new InstantCommand(() -> {
                m_intake.stopIntake();
            })
        );

        addRequirements(m_intake);
    }
}
