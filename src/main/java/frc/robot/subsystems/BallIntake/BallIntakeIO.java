package frc.robot.subsystems.ballIntake;

import org.littletonrobotics.junction.AutoLog;

public interface BallIntakeIO {

  @AutoLog
  public static class BallIntakeIOInputs {
    public double motorSpeed = 0.0;
  }

  public default void updateInputs(BallIntakeIOInputs inputs) {}

  public default void setMotorSpeed(double speed) {}
}
