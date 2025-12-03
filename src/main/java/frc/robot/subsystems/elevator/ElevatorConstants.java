package frc.robot.subsystems.elevator;

public class ElevatorConstants {

  public static final int elevatorMotorCurrentLimit = 60;

  public static final double positionConversionFactor = .0526;
  public static final double velocityConversionFactor = .0526;

  public class ElevatorSetpoints {
    public static final double StowPosition = 0.0;
    public static final double L1Position = 0.0;
    public static final double L2Position = 0.0;
    public static final double L3Position = 0.0;
    public static final double L4Position = 0.0;

    public static final double StowVelocity = 1.5;
    public static final double L1Velocity = 1.5;
    public static final double L2Velocity = 1.5;
    public static final double L3Velocity = 1.5;
    public static final double L4Velocity = 1.5;
  }

  public static class ControllerConstants {
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double velocityConstraint = 0.0;
    public static final double accelerationConstraint = 0.0;
    public static final double positionTolerance = 0.0;
    public static final double velocityTolerance = 0.0;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
  }

  public class ElevatorCANIds {
    public static final int leftMotorCANId = 5;
    public static final int rightMotorCANId = 6;
  }

  public static enum ElevatorStates {
    STOW(ElevatorSetpoints.StowPosition, ElevatorSetpoints.StowVelocity),
    L1(ElevatorSetpoints.L1Position, ElevatorSetpoints.L1Velocity),
    L2(ElevatorSetpoints.L2Position, ElevatorSetpoints.L2Velocity),
    L3(ElevatorSetpoints.L3Position, ElevatorSetpoints.L3Velocity),
    L4(ElevatorSetpoints.L4Position, ElevatorSetpoints.L4Velocity);

    public double elevatorSetpoint;
    public double elevatorVelocity;

    private ElevatorStates(double elevatorSetpoint, double elevatorVelocity) {
      this.elevatorSetpoint = elevatorSetpoint;
      this.elevatorVelocity = elevatorVelocity;
    }
  }
}
