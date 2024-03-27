// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRotate extends SequentialCommandGroup {
  /** Creates a new AutoRotate. */
  public AutoRotate(IntakeSubsystem m_robotIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeSetRotation(m_robotIntake, 0.55).withTimeout(0.65),
      new IntakeSetSpin(m_robotIntake, 0.8).withTimeout(0.5)
    );
  }
}
