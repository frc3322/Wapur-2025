package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double shooterMotorSpeed = 0.0;
    public double feederMotorSpeed = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterMotorSpeed(double speed) {}

  public default void setFeederMotorSpeed(double speed) {}
}
