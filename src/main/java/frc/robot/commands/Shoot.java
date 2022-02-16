package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends ParallelCommandGroup {
    public Shoot(ShooterSubsystem m_shooter, double runTime) {
        addCommands(
            sequence(
                new InstantCommand(() -> {
                m_shooter.plateDown();
                }),
                new WaitCommand(0.75),
                new FlipPlate(m_shooter),
                new WaitCommand(0.1),
                new TimedKick(m_shooter, 2),
                new WaitCommand(0.1),
                new FlipPlate(m_shooter)),
            new TimedShoot(m_shooter, runTime) //this should give the command group a max runtime of maxTime seconds :/
        );
    }
    
}
