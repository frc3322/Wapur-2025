package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Interface for elevator hardware abstraction layer. */
public interface ElevatorIO {

    /** Data class containing sensor inputs and control outputs from elevator. */
    @AutoLog
    public static class ElevatorIOInputs {
        public double motorPower = 0.0;

        public double position = 0.0;
        public double velocity = 0.0;

        public double setpoint = 0.0;
        public double pidOut = 0.0;
        public double ffOut = 0.0;

        public boolean atGoal = false;
    }

    public default void goToPosition(double positionMeters) {
    }

    public default void presetSetpoint(double setpointMeters) {
    }

    public default void zeroEncoder() {
    }

    public default void setMotorSpeeds(double speeds) {
    }

    public default void updateInputs(ElevatorIOInputs inputs) {
    }
}
