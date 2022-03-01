package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
public class FlipPlate extends SequentialCommandGroup{
    public FlipPlate(ShooterSubsystem m_shooter) {

        addCommands(
            new InstantCommand(() -> {
                m_shooter.plateUp();
            }),
            new WaitCommand(.15),
            new InstantCommand(() -> {
                m_shooter.plateDown();
            })
        );
    }
    
}
