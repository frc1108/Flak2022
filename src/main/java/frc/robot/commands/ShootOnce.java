package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootOnce extends ParallelCommandGroup {
    public ShootOnce(ShooterSubsystem m_shooter) {
        addCommands(
            sequence(
                new InstantCommand(() -> {
                m_shooter.plateDown();
                }),
                new WaitCommand(0.75),
                new FlipPlate(m_shooter)),
            new TimedShoot(m_shooter, 1.5) //this should give the command group a max runtime of maxTime seconds :/
        );
    }
    
}
