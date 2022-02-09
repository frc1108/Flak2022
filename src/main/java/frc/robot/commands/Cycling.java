package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
public class Cycling extends SequentialCommandGroup{
    public Cycling(ShooterSubsystem m_shooter) {

        addCommands(
            new InstantCommand(() -> {
                m_shooter.plateDown();
            }),
            new WaitCommand(0.3),
            //new TimedKick(m_shooter, 2),
            new WaitCommand(0.1),
            new FlipPlate(m_shooter),
            new WaitCommand(0.1),
            new TimedKick(m_shooter, 2),
            new WaitCommand(0.1),
            new FlipPlate(m_shooter)
        );
    }
    
}
