package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax IntakeSpinMotor;
    CANSparkMax IntakeRotateMotor;
    RelativeEncoder IntakeRotateEncoder;

    // Estimated kG
    // 5.06

    // Estimated kV
    // 0.23

    // Estimated kA
    // 0.14

    // Response Time
    // 0.58

    private final PIDController rotationPID = new PIDController(IntakeConstants.kP, IntakeConstants.kI,
            IntakeConstants.kD);

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Distance> m_angularPosition = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
    private final String INTAKE_SPIN_LOG_PATH = "/Spin";
    private final String INTAKE_ROTATION_LOG_PATH = "/Rotation/Speed";
    private final String INTAKE_ROTATION_ENCODER_LOG_PATH = "/Rotation/Encoder";
    private final String INTAKE_ROTATION_PID_LOG_PATH = "/Rotation/PIDSpeeds";

    SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
                IntakeRotateMotor.setVoltage(volts.in(Volts));
            }, log -> {
                // Record a frame for the left motors. Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("intake rotation")
                        .voltage(
                                m_appliedVoltage.mut_replace(
                                        IntakeRotateMotor.get() * RobotController.getBatteryVoltage(), Volts))
                        .linearPosition(m_angularPosition.mut_replace(IntakeRotateEncoder.getPosition(), Meters))
                        .linearVelocity(
                                m_velocity.mut_replace(IntakeRotateEncoder.getVelocity(), MetersPerSecond));
            },
                    // Tell SysId to make generated commands require this subsystem, suffix test
                    // state in
                    // WPILog with this subsystem's name ("drive")
                    this, "intake rotation"));

    public IntakeSubsystem() {
        IntakeSpinMotor = new CANSparkMax(IntakeConstants.IntakeSpinMotorCanID, MotorType.kBrushed);
        IntakeRotateMotor = new CANSparkMax(IntakeConstants.IntakeRotateMotorCanID, MotorType.kBrushless);
        IntakeRotateEncoder = IntakeRotateMotor.getEncoder();
        IntakeSpinMotor.setInverted(true);
        MathUtil.clamp(rotationPID.calculate(IntakeRotateEncoder.getPosition(), IntakeConstants.PIDSetpoint), -0.5,
                0.5);

    }

    public void setSpin(double speed) {
        IntakeSpinMotor.set(speed);
    }

    public void setRotate(double speed) {
        IntakeRotateMotor.set(speed);
    }

    public void setVoltage(double voltage){
        IntakeRotateMotor.setVoltage(4);
    }

    
    public void rotatePID() {
        IntakeRotateMotor.set(rotationPID.calculate(IntakeConstants.PIDSetpoint, IntakeRotateEncoder.getPosition()));
    }

    public Command ampPosition() {
        return run(()->
        IntakeRotateMotor.set(-rotationPID.calculate(IntakeConstants.PIDSetpoint, IntakeRotateEncoder.getPosition()))
        ).finallyDo(()->setRotate(0.0));
    }

    public void periodic() {
        SmartDashboard.putNumber("Rotation Encoder", getRotationEncoder());
        SmartDashboard.putNumber("Current Rotation Speed", getRotation());
        SmartDashboard.putNumber("Current Intake Speed", getSpin());
        SmartDashboard.putNumber("Rotation PID Speed",
                rotationPID.calculate(-IntakeConstants.PIDSetpoint, IntakeRotateEncoder.getPosition()));
        SmartDashboard.putData("Intake PID Data", rotationPID);
        SmartDashboard.putNumber("Sim Rotation Encoder", IntakeRotateEncoder.getPosition());
        logOutputs();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    /// Log IntakeSubsystem Outputs
    private void logOutputs() {
    Logger.recordOutput(getName() + INTAKE_SPIN_LOG_PATH, getSpin());
    Logger.recordOutput(getName() + INTAKE_ROTATION_LOG_PATH, getRotation());
    Logger.recordOutput(getName() + INTAKE_ROTATION_ENCODER_LOG_PATH, getRotationEncoder());
    Logger.recordOutput(getName() + INTAKE_ROTATION_PID_LOG_PATH, rotationPID.calculate(IntakeConstants.PIDSetpoint, IntakeRotateEncoder.getPosition()));
    }

    public double getSpin(){
        return IntakeSpinMotor.get();
    }

    public double getRotation(){
        return IntakeRotateMotor.get();
    }

    public double getRotationEncoder(){
        return IntakeRotateEncoder.getPosition();
        //return -3;
    }
}
