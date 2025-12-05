package frc.robot.subsystems.dropper;

import org.littletonrobotics.junction.AutoLog;

public interface DropperIO {

  @AutoLog
  public static class dropperIOInputs {
    public double dropperMotorSpeed = 0.0;
  }

  // Sets motor speed
  public default void setMotorSpeeds(double speeds) {}

  public default void updateInputs(dropperIOInputs inputs) {}
}
