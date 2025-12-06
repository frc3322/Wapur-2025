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
        public static final int leftMotorCANId = 55;
        public static final int rightMotorCANId = 35;
    }

    public static enum ElevatorStates {
        STOW(ElevatorSetpoints.StowPosition),
        L1(ElevatorSetpoints.L1Position),
        L2(ElevatorSetpoints.L2Position),
        L3(ElevatorSetpoints.L3Position),
        L4(ElevatorSetpoints.L4Position);

        public double elevatorSetpoint;

        private ElevatorStates(double elevatorSetpoint) {
            this.elevatorSetpoint = elevatorSetpoint;
        }
    }
}
