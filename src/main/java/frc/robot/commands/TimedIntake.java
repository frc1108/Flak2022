package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
public class TimedIntake extends SequentialCommandGroup{
    public TimedIntake(IntakeSubsystem m_intake, double seconds) {
        addCommands(
            new RunCommand(() -> {
                m_intake.intake(12);
            }).withTimeout(seconds),
            new InstantCommand(() -> {
                m_intake.stopIntake();
            })
        );
    }
}
