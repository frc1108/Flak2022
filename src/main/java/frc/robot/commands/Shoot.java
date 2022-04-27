package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;


public class Shoot extends ParallelRaceGroup {
    /**
   * Command that shoots twice and then stops the shooter wheels.
   * Has an optional speed modifier parameter (double)
   * @param m_shooter the subsystem obviously
   * @param powerModifierPercent the additional percent to add or subtract to the shooter
   */
    private double kickTime = 0.83;
    public Shoot(ShooterSubsystem m_shooter) {
        addCommands(
            sequence(
                new InstantCommand(() -> {
                m_shooter.plateDown();
                }),
                new TimedKick(m_shooter, 0.1, -0.55),
                new WaitCommand(0.35),
                new FlipPlate(m_shooter),
                new WaitCommand(0.1),
                new TimedKick(m_shooter, kickTime),
                new WaitCommand(0.2),
                new FlipPlate(m_shooter)),
            //new TimedShoot(m_shooter, runTime), //this should give the command group a max runtime of maxTime seconds :/
            new RunCommand(()->m_shooter.shoot())
            .andThen(new InstantCommand(()->m_shooter.stopShoot()))
            );
        addRequirements(m_shooter);
    }
    public Shoot(ShooterSubsystem m_shooter, double extraKickTime) {
        addCommands(
            sequence(
                new InstantCommand(() -> {
                m_shooter.plateDown();
                }),
                new TimedKick(m_shooter, 0.1, -0.55),
                new WaitCommand(0.35),
                new FlipPlate(m_shooter),
                new WaitCommand(0.1),
                new TimedKick(m_shooter, kickTime+extraKickTime),
                new WaitCommand(0.2),
                new FlipPlate(m_shooter)),
            //new TimedShoot(m_shooter, runTime), //this should give the command group a max runtime of maxTime seconds :/
            new RunCommand(()->m_shooter.shoot())
            .andThen(new InstantCommand(()->m_shooter.stopShoot()))
            );
        addRequirements(m_shooter);
    }

}
