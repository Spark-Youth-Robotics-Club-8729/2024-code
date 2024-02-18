// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShooterSet;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShot extends SequentialCommandGroup {
  /** Creates a new AutoShot. */
  public AutoShot(ShooterSubsystem m_robotShooter, IntakeSubsystem m_robotIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterSet(m_robotShooter, 0.9).withTimeout(0.4),
      new ParallelCommandGroup (
        new ShooterSet(m_robotShooter, 0.9).withTimeout(0.6),
        new IntakeSetSpin(m_robotIntake, 0.6).withTimeout(0.6)
      )
    );
  }
}
