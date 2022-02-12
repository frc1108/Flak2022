package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends ParallelCommandGroup {
    public Shoot(ShooterSubsystem m_shooter, double percent) {
        addCommands(
            new Cycling(m_shooter).andThen(new WaitCommand(1)),
            new TimedShoot(m_shooter, percent, 10) //this should give the command group a max runtime of 10 seconds :/
        );
    }
    public Shoot(ShooterSubsystem m_shooter, double percent, double runTime) {
        addCommands(
            new Cycling(m_shooter).andThen(new WaitCommand(1)),
            new TimedShoot(m_shooter, percent, runTime) //this should give the command group a max runtime of maxTime seconds :/
        );
    }
    
}
