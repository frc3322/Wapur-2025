package frc.robot.subsystems.crateIntake;

import org.littletonrobotics.junction.AutoLog;

/** Interface for crate intake hardware abstraction layer. */
public interface CrateIntakeIO {

  /** Data class containing sensor inputs from the crate intake. */
  @AutoLog
  public static class CrateIntakeIOInputs {
    public double leftMotorSpeed = 0.0;
    public double rightMotorSpeed = 0.0;
  }

  public default void updateInputs(CrateIntakeIOInputs inputs) {}

  public default void setMotorSpeed(double speed) {}
}
