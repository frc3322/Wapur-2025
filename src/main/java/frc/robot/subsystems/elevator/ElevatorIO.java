package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog // Logging the inputs
    public class ElevatorIOInputs {
        // Motor powers for both motors
        public double leftMotorPower = 0.0;
        public double rightMotorPower = 0.0;

        // Position and Velocity of elevator
        public double position = 0.0;
        public double velocity = 0.0;

        // PID, FeedForward, and Setpoint
        public double pidOut = 0.0;
        public double ffOut = 0.0;
        public double setpoint = 0.0;

        // At goal
        public boolean atGoal = false;
    }

    /**
     * Lets us move the elevator to a specific position
     *
     * @param PositionInMeters
     */
    public default void goToPosition(double PositionInMeters) {
    }

    public default void presetSetpoint(double SetpointInMeters) {
    }

    // Zeros the encoder, pretty self explanatory
    public default void zeroEncoder() {
    }

    // Sets motor speed
    public default void setMotorSpeeds(double speeds) {
    }

    // Updates the inputs in the class above
    public default void updateInputs(ElevatorIOInputs inputs) {
    }
}
