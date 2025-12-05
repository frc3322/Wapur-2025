package frc.robot.subsystems.elevator;

/**
 * Constants for the elevator subsystem including motor config, PID gains, and
 * setpoints.
 */
public class ElevatorConstants {
    public static final int motorCanId = 6;
    public static final int motorCurrentLimit = 50;

    public static final double positionConversionFactor = 0.0526;
    public static final double velocityConversionFactor = 0.0526;

    /** PID controller and feedforward constants for elevator control. */
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

    /** Predefined elevator position setpoints in meters. */
    public static class ElevatorSetpoints {
        public static final double intakePosition = 0.1;
        public static final double level0Position = 0.0;
        public static final double level1Position = 0.3;
        public static final double level2Position = 0.6;
        public static final double level3Position = 1.0;
    }

    /** Enum representing possible elevator positions/states. */
    public enum ElevatorState {
        INTAKE(ElevatorSetpoints.intakePosition),
        LEVEL0(ElevatorSetpoints.level0Position),
        LEVEL1(ElevatorSetpoints.level1Position),
        LEVEL2(ElevatorSetpoints.level2Position),
        LEVEL3(ElevatorSetpoints.level3Position);

        public double elevatorSetpoint;

        ElevatorState(double elevatorSetpoint) {
            this.elevatorSetpoint = elevatorSetpoint;
        }
    }
}
