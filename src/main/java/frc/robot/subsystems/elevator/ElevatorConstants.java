package frc.robot.subsystems.elevator;

public class ElevatorConstants {
  public static final int motorCanId = 6;
  public static final int motorCurrentLimit = 50;

  public static final double positionConversionFactor = 0.0526;
  public static final double velocityConversionFactor = 0.0526;

  public static class ControllerConstants {
    public static final double kP = 2.0;
    public static final double kI = 0.1;
    public static final double kD = 0.0;
    public static final double velocityConstraint = 3.0;
    public static final double accelerationConstraint = 4.0;
    public static final double positionTolerance = 0.05;
    public static final double velocityTolerance = 0.05;

    public static final double kS = 0.025;
    public static final double kG = 0.035;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
  }

  public static class ElevatorSetpoints {
    public static final double stowPosition = 0.0;
    public static final double upPosition = 1.0;
    public static final double downPosition = 0.0;

    public static final double stowVelocity = 0.0;
    public static final double upVelocity = 0.0;
    public static final double downVelocity = 0.0;
  }

  public enum ElevatorState {
    STOW(ElevatorSetpoints.stowPosition, ElevatorSetpoints.stowVelocity),
    UP(ElevatorSetpoints.upPosition, ElevatorSetpoints.upVelocity),
    DOWN(ElevatorSetpoints.downPosition, ElevatorSetpoints.downVelocity);

    public double elevatorSetpoint;
    public double elevatorVelocity;

    ElevatorState(double elevatorSetpoint, double elevatorVelocity) {
      this.elevatorSetpoint = elevatorSetpoint;
      this.elevatorVelocity = elevatorVelocity;
    }
  }
}
