package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.elevator.ElevatorConstants.ControllerConstants;

/** Real hardware implementation for elevator using SparkFlex motor with PID and feedforward control. */
public class ElevatorIOReal implements ElevatorIO {
    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    private final ProfiledPIDController elevatorPID;
    private final ArmFeedforward elevatorFeedforward;

    private double pidOutput;
    private double ffOutput;

    /** Initializes elevator motor with proper configuration and controllers. */
    public ElevatorIOReal() {
        motor = new SparkFlex(ElevatorConstants.motorCanId, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.motorCurrentLimit)
                .voltageCompensation(12);
        config.encoder
                .positionConversionFactor(ElevatorConstants.positionConversionFactor)
                .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);

        tryUntilOk(
                motor,
                5,
                () -> motor.configure(
                        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        encoder = motor.getEncoder();

        elevatorPID = new ProfiledPIDController(
                ControllerConstants.kP,
                ControllerConstants.kI,
                ControllerConstants.kD,
                new Constraints(
                        ControllerConstants.velocityConstraint,
                        ControllerConstants.accelerationConstraint));

        elevatorPID.setTolerance(
                ControllerConstants.positionTolerance, ControllerConstants.velocityTolerance);

        elevatorFeedforward = new ArmFeedforward(
                ControllerConstants.kS,
                ControllerConstants.kG,
                ControllerConstants.kV,
                ControllerConstants.kA);
    }

    /** Sets PID controller goal to specified position in rotations. */
    @Override
    public void goToPosition(double positionRotations) {
        elevatorPID.setGoal(positionRotations);
    }

    /** Presets PID controller goal without moving (used for state setting). */
    @Override
    public void presetSetpoint(double setpointMeters) {
        elevatorPID.setGoal(setpointMeters);
    }

    /** Zeros the encoder position to current location. */
    @Override
    public void zeroEncoder() {
        encoder.setPosition(0);
    }

    /** Sets raw motor speed (-1.0 to 1.0). */
    @Override
    public void setMotorSpeeds(double speeds) {
        motor.set(speeds);
    }

    /** Updates PID and feedforward control outputs and applies to motor. */
    private void updateControl() {
        var setpoint = elevatorPID.getSetpoint();
        ffOutput = elevatorFeedforward.calculate(setpoint.position, setpoint.velocity);
        pidOutput = elevatorPID.calculate(encoder.getPosition());

        double combinedOutput = ffOutput + pidOutput;
        motor.set(combinedOutput);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();

        // Update control outputs continuously
        updateControl();

        inputs.motorPower = motor.get();

        inputs.setpoint = elevatorPID.getGoal().position;
        inputs.pidOut = pidOutput;
        inputs.ffOut = ffOutput;

        inputs.atGoal = elevatorPID.atGoal();
    }
}
