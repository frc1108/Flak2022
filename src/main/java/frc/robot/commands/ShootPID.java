// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPID extends ParallelRaceGroup {
  /** Creates a new ShootPID. */
  public ShootPID(ShooterSubsystem m_shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(m_shooter::startShooter),
      sequence(
        new InstantCommand(() -> {
        m_shooter.plateDown();
        }),
        new WaitUntilCommand(()->m_shooter.atSpeed()).withTimeout(0.75),
        new FlipPlate(m_shooter),
        new WaitCommand(0.1),
        new TimedKick(m_shooter, 2),
        new WaitUntilCommand(()->m_shooter.atSpeed()).withTimeout(0.1),
        new FlipPlate(m_shooter)));
  }
}
