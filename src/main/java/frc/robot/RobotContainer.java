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
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClimberSet;
import frc.robot.commands.IntakeSetRotation;
import frc.robot.commands.IntakeSetSpin;
import frc.robot.commands.ShooterSet;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoShot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoRevShooter;
import frc.robot.commands.AutoRotate;
import frc.robot.commands.IntakeAndRetract;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.ServoSetRotation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

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
        private final ServoSubsystem m_robotServo = new ServoSubsystem();


        private final Field2d field;

        Pose2d allianceWingTargetPose = new Pose2d(5.62, 6.72, Rotation2d.fromDegrees(0.0));

        Pose2d subwooferPose = new Pose2d(1.24, 5.55, Rotation2d.fromDegrees(0.0));

        // Load the paths we want to follow
        PathPlannerPath ampSideWingToSubwoofer = PathPlannerPath.fromPathFile("WingToSubwoofer");
        PathPlannerPath sourceSideWingToSubwoofer = PathPlannerPath.fromPathFile("SourceToSubwoofer");
        PathPlannerPath toAllianceSource = PathPlannerPath.fromPathFile("ToSource");

        // Create the constraints to use while pathfinding. The constraints defined in
        // the path will only be used for the path.
        PathConstraints constraintsA = new PathConstraints(
                        3.0, 3.0,
                        2 * Math.PI, 2 * Math.PI);

        private static SendableChooser<Command> m_autoChooser = new SendableChooser<>();

        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

        // The operator's controller
        CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                NamedCommands.registerCommand("AutoShot", new AutoShot(m_robotShooter, m_robotIntake));
                NamedCommands.registerCommand("AutoIntake", new AutoIntake(m_robotIntake));
                NamedCommands.registerCommand("AutoRotate", new AutoRotate(m_robotIntake));
                NamedCommands.registerCommand("AutoRevShooter", new AutoRevShooter(m_robotShooter));

                // Configure the button bindings
                configureButtonBindings();

                // Configure autochooser
                autoChooser();

                // Smartdashboard subsystem data
                SmartDashboard.putData(m_robotDrive);
                SmartDashboard.putData(m_robotIntake);
                SmartDashboard.putData(m_robotShooter);
                SmartDashboard.putData(m_robotClimb);

                field = new Field2d();
                SmartDashboard.putData("Field", field);

                // Logging callback for current robot pose
                PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        field.setRobotPose(pose);
                });

                // Logging callback for target robot pose
                PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
                        // Do whatever you want with the pose here
                        field.getObject("target pose").setPose(pose);
                });

                // Logging callback for the active path, this is sent as a list of poses
                PathPlannerLogging.setLogActivePathCallback((poses) -> {
                        // Do whatever you want with the poses here
                        field.getObject("path").setPoses(poses);
                });

                // Since AutoBuilder is configured, we can use it to build pathfinding commands

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
                Logger.recordOutput("Raw Driver Data axis 1", m_driverController.getRawAxis(1));
                Logger.recordOutput("Raw Driver Data axis 0", m_driverController.getRawAxis(0));
                Logger.recordOutput("Raw Driver Data axis 4", m_driverController.getRawAxis(4));
                Logger.recordOutput("Processed Driver Data axis 1", -MathUtil.applyDeadband(
                                m_driverController.getRawAxis(1),
                                OIConstants.kDriveDeadband));
                Logger.recordOutput("Processed Driver Data axis 0", -MathUtil.applyDeadband(
                                m_driverController.getRawAxis(0),
                                OIConstants.kDriveDeadband));
                Logger.recordOutput("Processed Driver Data axis 4", -MathUtil.applyDeadband(
                                m_driverController.getRawAxis(4),
                                OIConstants.kDriveDeadband));

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
                m_driverController.leftBumper()
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));

                m_driverController.povLeft()
                                .whileTrue(new RunCommand(() -> m_robotDrive.m_gyro.zeroYaw(), m_robotDrive));

                ////

                m_driverController.leftTrigger(0.1)
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRawAxis(1),
                                                                                OIConstants.kDriveDeadband)
                                                                                * m_driverController
                                                                                                .getLeftTriggerAxis()
                                                                                * 0.4,
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRawAxis(0),
                                                                                OIConstants.kDriveDeadband)
                                                                                * m_driverController
                                                                                                .getLeftTriggerAxis()
                                                                                * 0.4,
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRawAxis(4),
                                                                                OIConstants.kDriveDeadband)
                                                                                * m_driverController
                                                                                                .getLeftTriggerAxis()
                                                                                * 0.4,
                                                                true, true),
                                                m_robotDrive));

                m_driverController.x()
                                .whileTrue(
                                                AutoBuilder.pathfindThenFollowPath(
                                                                ampSideWingToSubwoofer, constraintsA, 1.4 // Rotation
                                                                                                          // delay
                                                                                                          // distance in
                                                                                                          // meters.
                                                                                                          // This is how
                                                                                                          // far the
                                                                                                          // robot
                                                                                                          // should
                                                                                                          // travel
                                                                                                          // before
                                                                                                          // attempting
                                                                                                          // to rotate.
                                                ));
                m_driverController.a().whileTrue(
                                AutoBuilder.pathfindThenFollowPath(
                                                sourceSideWingToSubwoofer, constraintsA, 1.4 // Rotation delay distance
                                                                                             // in meters. This is how
                                                                                             // far the robot should
                                                                                             // travel before attempting
                                                                                             // to rotate.
                                ));
                m_driverController.y().whileTrue(
                                AutoBuilder.pathfindThenFollowPath(
                                                toAllianceSource, constraintsA, 1.4 // Rotation delay distance in
                                                                                    // meters. This is how far the robot
                                                                                    // should travel before attempting
                                                                                    // to rotate.
                                ));

                ///

                // m_driverController.povUp()
                // .whileTrue(new ClimberSet(m_robotClimb, -0.9));
                // m_driverController.povDown()
                // .whileTrue(new ClimberSet(m_robotClimb, 0.9));
                // m_driverController.b().whileTrue(new IntakeSetSpin(m_robotIntake, 0.9));
                // m_driverController.x().whileTrue(new IntakeSetSpin(m_robotIntake, -0.8));
                // m_driverController.rightBumper().whileTrue(new ShooterSet(m_robotShooter,
                // 0.9, true));
                // m_driverController.leftTrigger().whileTrue(new ShooterSet(m_robotShooter,
                // -0.25, true));
                // m_driverController.rightTrigger().whileTrue(new ShooterSet(m_robotShooter,
                // 0.17, false));

                // m_driverController.y().whileTrue(new IntakeSetRotation(m_robotIntake, 0.3));
                // m_driverController.a().whileTrue(new IntakeSetRotation(m_robotIntake, -0.3));
                // m_operatorController.povRight()
                // .whileTrue(new IntakeAndRetract(m_robotIntake, -0.3, -0.8))// rot out speed,
                // intake
                // .onFalse(m_robotIntake.ampPosition().withTimeout(1.2));
                // m_driverController.povLeft().whileTrue(m_robotIntake.ampPosition());

                m_operatorController.povUp()
                                .whileTrue(new ClimberSet(m_robotClimb, -0.9));
                m_operatorController.povDown()
                                .whileTrue(new ClimberSet(m_robotClimb, 0.9));
                m_operatorController.b().whileTrue(new IntakeSetSpin(m_robotIntake, -0.9));
                m_operatorController.x().whileTrue(new IntakeSetSpin(m_robotIntake, 0.7));
                m_operatorController.rightBumper().whileTrue(new ShooterSet(m_robotShooter, 0.9, true));
                m_operatorController.leftBumper().whileTrue(new ShooterSet(m_robotShooter, -0.25, true));
                m_operatorController.rightTrigger().whileTrue(new IntakeSetSpin(m_robotIntake, -0.613));
                // m_operatorController.povLeft().whileTrue(m_robotIntake.ampPosition());
                m_operatorController.y().whileTrue(new IntakeSetRotation(m_robotIntake, 0.5));
                m_operatorController.a().whileTrue(new IntakeSetRotation(m_robotIntake, -0.5));
                m_operatorController.povRight().whileTrue(new ServoSetRotation(m_robotServo, 180));

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
         * Add auto sequences to chooser
         */
        private void autoChooser() {

                /*
                 * ////////////////////NOTE MAPPINGS////////////////////
                 * |____|<--amp /
                 * center1
                 * AmpSide close1 /
                 * \ /
                 * | center2
                 * // sub->* |Center close2 /
                 * | /
                 * / center3
                 * SourceSide close3 [stage area] /
                 * /
                 * center4
                 * /
                 * /
                 * --- center5
                 * //source* \ /
                 *//////////////////////////////////////////////////////

                m_autoChooser.addOption("AmpSide: Score preloaded + do nothing",
                                new PathPlannerAuto("UpperSubScorePreloadOnly"));
                m_autoChooser.addOption("AmpSide: Close1 + Center2,1", new PathPlannerAuto("UpperSubClose1Center21"));
                m_autoChooser.addOption("AmpSide: Close1,2 + Center1", new PathPlannerAuto("UpperSubClose12Center1"));
                m_autoChooser.addOption("Ampside: Troll Centerline Auto", new PathPlannerAuto("UpperSubTrollAuto"));

                m_autoChooser.addOption("Center: Score preloaded + do nothing",
                                new PathPlannerAuto("MidSubScorePreloadOnly"));
                m_autoChooser.addOption("Center: Close2,3,1 with rotation",
                                new PathPlannerAuto("MidSubClose231Rotation"));
                m_autoChooser.setDefaultOption("Center: Close2,3,1 no rotation",
                                new PathPlannerAuto("MidSubClose231NoRotation"));
                m_autoChooser.addOption("Center: Close2,3 + Center 2", new PathPlannerAuto("MidSubClose23Center2"));
                m_autoChooser.addOption("Center: Close2,3 + Center 1", new PathPlannerAuto("MidSubClose23Center1"));
                m_autoChooser.addOption("Center: Close 2,3 + Center 4", new PathPlannerAuto("MidSubClose23Center4"));

                m_autoChooser.addOption("SourceSide: Score preloaded + do nothing",
                                new PathPlannerAuto("LowerSubScorePreloadOnly"));
                m_autoChooser.addOption("SourceSide: Close3 + Center4,5",
                                new PathPlannerAuto("LowerSubClose3Center45"));
                m_autoChooser.addOption("SourceSide: Center5,4 + Close3",
                                new PathPlannerAuto("LowerSubCenter54Close3"));
                m_autoChooser.addOption("SourceSide: Straight center5,4 + close3",
                                new PathPlannerAuto("LowerSubCenter5Straight"));
                m_autoChooser.addOption("SourceSide: Troll Centerline Auto", new PathPlannerAuto("LowerSubTrollAuto"));
                m_autoChooser.addOption("SourceSide: Score preloaded + troll centerline",
                                new PathPlannerAuto("LowerSubLongTrollAuto"));
                m_autoChooser.addOption("Test-Center: Mid + Amp side 3 note auto",
                                new PathPlannerAuto("MidShotNoteShot"));
                m_autoChooser.addOption("Test-Command: Auto Rev Shooter", new PathPlannerAuto("AutoRevShooterTest"));
                m_autoChooser.addOption("Test-Command: Auto Shot", new PathPlannerAuto("AutoShotTest"));
                m_autoChooser.addOption("Test-Command: Auto Intake", new PathPlannerAuto("AutoIntakeTest"));
                m_autoChooser.addOption("Test-Command: Auto Rotate", new PathPlannerAuto("AutoRotateTest"));
        m_autoChooser.addOption("Troll1",
                                new PathPlannerAuto("GriefCenterAuto"));
                // m_autoChooser.addOption("4 Note Auto 1", new PathPlannerAuto("4 Note Auto
                // 1"));

                SmartDashboard.putData("Auto Chooser", m_autoChooser);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return m_autoChooser.getSelected();
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
