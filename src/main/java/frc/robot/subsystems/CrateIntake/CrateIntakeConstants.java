package frc.robot.subsystems.crateIntake;

/** Constants for the crate intake subsystem including motor IDs and limits. */
public class CrateIntakeConstants {
  public static final int leftMotorCanId = 3;
  public static final int rightMotorCanId = 4;
  public static final int motorCurrentLimit = 40;

  /** Enum representing possible states of the crate intake mechanism. */
  public enum CrateIntakeState {
    INTAKE(1.0),  // Intake crates inward
    STOP(0.0),    // Stop motors
    OUTTAKE(-1.0); // Push crates outward

    public final double motorSpeed;

    CrateIntakeState(double motorSpeed) {
      this.motorSpeed = motorSpeed;
    }
  }
}
