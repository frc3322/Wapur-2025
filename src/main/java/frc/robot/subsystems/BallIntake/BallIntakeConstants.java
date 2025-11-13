package frc.robot.subsystems.ballIntake;

public class BallIntakeConstants {
  public static final int motorCanId = 5;
  public static final int motorCurrentLimit = 40;

  public enum BallIntakeState {
    INTAKE(1.0),
    STOP(0.0),
    OUTTAKE(-1.0);

    public final double motorSpeed;

    BallIntakeState(double motorSpeed) {
      this.motorSpeed = motorSpeed;
    }
  }
}
