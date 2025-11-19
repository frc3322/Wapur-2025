package frc.robot.subsystems.BallIntake; // Packages are consisten with the folder name

import org.littletonrobotics.junction.AutoLog;

public interface BallIntakeIO // Classes and methods are all capitalized
{
    @AutoLog
    public static class BallIntakeIOInputs
    {
        public double motorSpeed = 0.0;
    }

    public default void updateInputs(BallIntakeIOInputs inputs) {}
    public default void setMotorspeed(double speed) {}
}
