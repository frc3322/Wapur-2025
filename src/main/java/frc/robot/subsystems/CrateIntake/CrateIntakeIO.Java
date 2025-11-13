package frc.tobot.subsystems.CrateIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CrateIntakeIO
{
    @AutoLog
    public static class CrateIntakeIOInputs
    {
        public double leftMotorSpeed = 0.0;
        public double rightMotorSpeed = 0.0;
    }

    public default void updateInputs(CrateIntakeIOInputs inputs)
    {
        // nothing
    }
    public default void setMotorspeed(double speed)
    {
        // nothing. Default keyword lets the code compile wiothout error even if no arguments are in the method
    }
}