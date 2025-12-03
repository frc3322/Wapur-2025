package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.elevator.ElevatorConstants.ControllerConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorCANIds;

public class ElevatorIOReal implements ElevatorIO {
  private SparkFlex leftMotor;

  RelativeEncoder LeftEncoder;

  private ProfiledPIDController elevatorPID;
  private ArmFeedforward elevatorFeedForward;

  private double pidOut;
  private double ffOut;

  public ElevatorIOReal() {
    // Initializeing the motors
    leftMotor = new SparkFlex(ElevatorCANIds.leftMotorCANId, MotorType.kBrushless);

    // The setting objects for both motors
    SparkMaxConfig leftConfig = new SparkMaxConfig();

    // Left motor settings
    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.elevatorMotorCurrentLimit)
        .voltageCompensation(12)
        .inverted(false);
    leftConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);

    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftConfig,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kPersistParameters));

    LeftEncoder = leftMotor.getEncoder();

    elevatorPID =
        new ProfiledPIDController(
            ControllerConstants.kP,
            ControllerConstants.kI,
            ControllerConstants.kD,
            new Constraints(
                ControllerConstants.velocityConstraint,
                ControllerConstants.accelerationConstraint));

    elevatorPID.setTolerance(
        ControllerConstants.positionTolerance, ControllerConstants.velocityTolerance);

    elevatorFeedForward =
        new ArmFeedforward(
            ControllerConstants.kS,
            ControllerConstants.kG,
            ControllerConstants.kV,
            ControllerConstants.kA);
  }

  @Override
  public void goToPosition(double positionRotations, double velocityRotPerSec) {
    ffOut = elevatorFeedForward.calculate(positionRotations, velocityRotPerSec);

    elevatorPID.setGoal(positionRotations);
    pidOut = elevatorPID.calculate(LeftEncoder.getPosition());

    double combinedOutput = ffOut + pidOut;

    leftMotor.set(combinedOutput);
  }

  @Override
  public void presetSetpoint(double setpointMeters) {
    elevatorPID.setGoal(setpointMeters);
  }

  @Override
  public void zeroEncoder() {
    LeftEncoder.setPosition(0);
  }

  @Override
  public void setMotorSpeeds(double speeds) {
    leftMotor.set(speeds);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.position = LeftEncoder.getPosition();
    inputs.velocity = LeftEncoder.getVelocity();

    inputs.leftMotorPower = leftMotor.get();

    inputs.setpoint = elevatorPID.getGoal().position;
    inputs.pidOut = pidOut;
    inputs.ffOut = ffOut;

    inputs.atGoal = elevatorPID.atGoal();
  }
}
