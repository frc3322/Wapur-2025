package frc.robot.subsystems.Dropper;

import org.littletonrobotics.junction.AutoLog;

public interface DropperIO 
{

    @AutoLog
    public static class dropperIOInputs() {
        public double dropperMotorSpeed = 0.0;
    }

    // Sets motor speed
    public default void setMotorSpeeds(double speeds) {}
}
