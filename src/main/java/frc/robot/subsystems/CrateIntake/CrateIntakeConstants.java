package frc.robot.subsystems.crateIntake;

public class CrateIntakeConstants {
  public static final int leftMotorCanId = 3;
  public static final int rightMotorCanId = 4;
  public static final int motorCurrentLimit = 40;

  public enum CrateIntakeState {
    INTAKE(1.0),
    STOP(0.0),
    OUTTAKE(-1.0);

    public final double motorSpeed;

    CrateIntakeState(double motorSpeed) {
      this.motorSpeed = motorSpeed;
    }
  }
}
