// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterSet extends Command {
  ShooterSubsystem m_shooterSubsystem;
  double m_speed;
  boolean spin;

  /** Creates a new ShooterSet. */
  public ShooterSet(ShooterSubsystem shooterSubsystem, double speed, boolean addSpin) {
    m_shooterSubsystem = shooterSubsystem;
    m_speed = speed;
    spin = addSpin;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(spin){
      m_shooterSubsystem.setShooter(m_speed, m_speed-0.15);
    }
    else{
      m_shooterSubsystem.setShooter(m_speed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setShooter(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
