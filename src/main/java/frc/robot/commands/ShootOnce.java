package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootOnce extends ParallelRaceGroup {
    /**
   * Command that shoots one ball and then stops the shooter wheels.
   * Has an optional speed modifier paramater (double)
   * @param m_shooter the subsystem obviously
   * @param powerModifierPercent the additional percent to add or subtract to the shooter
   */
    public ShootOnce(ShooterSubsystem m_shooter) {
        addCommands(
            sequence(
                new InstantCommand(() -> {
                m_shooter.plateDown();
                }),
                new WaitCommand(0.75),
                new FlipPlate(m_shooter)),
            new RunCommand(()->m_shooter.shoot())
            .andThen(new InstantCommand(()->m_shooter.stopShoot())));
        addRequirements(m_shooter);
    }
}
