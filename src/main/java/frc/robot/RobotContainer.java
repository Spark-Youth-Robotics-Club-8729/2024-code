// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ClimberSet;
import frc.robot.commands.IntakeSetRotation;
import frc.robot.commands.IntakeSetSpin;
import frc.robot.commands.ShooterSet;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoShot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoRotate;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();
        private final ClimbSubsystem m_robotClimb = new ClimbSubsystem();
        private final ShooterSubsystem m_robotShooter = new ShooterSubsystem();



        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

        // The intake joystick
        CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                NamedCommands.registerCommand("AutoShot", new AutoShot(m_robotShooter, m_robotIntake));
                NamedCommands.registerCommand("AutoIntake", new AutoIntake(m_robotIntake));
                NamedCommands.registerCommand("AutoRotate", new AutoRotate(m_robotIntake));

                
                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRawAxis(1),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRawAxis(0),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRawAxis(4),
                                                                                OIConstants.kDriveDeadband),
                                                                true, true),
                                                m_robotDrive));

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link   
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link JoystickButton}.
         */
        private void configureButtonBindings() {
                m_driverController.rightBumper()
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));
                m_driverController.y().whileTrue(new RunCommand(() -> m_robotDrive.m_gyro.zeroYaw(), m_robotDrive));

                // new JoystickButton(m_operatorController, 9).whileTrue(
                // new RunCommand(() ->
                // m_robotClimb.setSpin(m_operatorController.getRawAxis(1)), m_robotClimb));
                m_operatorController.leftTrigger()
                                .whileTrue(new ClimberSet(m_robotClimb, m_operatorController.getLeftY()));
                m_operatorController.b().whileTrue(new IntakeSetSpin(m_robotIntake, 0.6));
                m_operatorController.x().whileTrue(new IntakeSetSpin(m_robotIntake, -0.8));
                m_operatorController.rightBumper().whileTrue(new ShooterSet(m_robotShooter, 0.9));
                m_operatorController.leftBumper().whileTrue(new ShooterSet(m_robotShooter, -0.5));
                m_operatorController.rightTrigger().whileTrue(new ShooterSet(m_robotShooter, 0.5));
                m_operatorController.y().whileTrue(new IntakeSetRotation(m_robotIntake, 0.3));
                m_operatorController.a().whileTrue(new IntakeSetRotation(m_robotIntake, -0.3));

                // new JoystickButton(m_operatorController, 1)
                // .whileTrue(new IntakeSetRotation(m_robotIntake, 0.5));
                // new JoystickButton(m_operatorController, 4)
                // .whileTrue(new IntakeSetRotation(m_robotIntake, -0.5));

                // new JoystickButton(m_operatorController, 7)
                // .whileTrue(new ShooterSet(m_robotShooter, 0.5));
                // new JoystickButton(m_operatorController, 8)
                // .whileTrue(new ShooterSet(m_robotShooter, -0.5));
                // new JoystickButton(m_driverController, 1)
                // .whileTrue(m_robotIntake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

                // new JoystickButton(m_driverController, 2)
                // .whileTrue(m_robotIntake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

                // new JoystickButton(m_driverController, 3)
                // .whileTrue(m_robotIntake.sysIdDynamic(SysIdRoutine.Direction.kForward));

                // new JoystickButton(m_driverController, 4)
                // .whileTrue(m_robotIntake.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        public void resetGyroCommand() {
                m_robotDrive.m_gyro.zeroYaw();
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new PathPlannerAuto("MidShotNoteShot");
                // // Create config for trajectory
                // TrajectoryConfig config = new TrajectoryConfig(
                // AutoConstants.kMaxSpeedMetersPerSecond,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // // Add kinematics to ensure max speed is actually obeyed
                // .setKinematics(DriveConstants.kDriveKinematics);

                // // An example trajectory to follow. All units in meters.
                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(3, 0, new Rotation2d(0)),
                // config);

                // var thetaController = new ProfiledPIDController(
                // AutoConstants.kPThetaController, 0, 0,
                // AutoConstants.kThetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // exampleTrajectory,
                // m_robotDrive::getPose, // Functional interface to feed supplier
                // DriveConstants.kDriveKinematics,

                // // Position controllers
                // new PIDController(AutoConstants.kPXController, 0, 0),
                // new PIDController(AutoConstants.kPYController, 0, 0),
                // thetaController,
                // m_robotDrive::setModuleStates,
                // m_robotDrive);

                // // Reset odometry to the starting pose of the trajectory.
                // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // // Run path following command, then stop at the end.
                // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
                // false, false));
        }
}
