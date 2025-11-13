package frc.robot.subsystems.crateIntake;

import org.littletonrobotics.junction.AutoLog;

public interface CrateIntakeIO {

  @AutoLog
  public static class CrateIntakeIOInputs {
    public double leftMotorSpeed = 0.0;
    public double rightMotorSpeed = 0.0;
  }

  public default void updateInputs(CrateIntakeIOInputs inputs) {}

  public default void setMotorSpeed(double speed) {}
}
