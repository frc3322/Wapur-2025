package frc.robot.subsystems.shooter;

public class ShooterConstants {
  public static final int shooterMotorCanId = 1;
  public static final int feederMotorCanId = 2;
  public static final int shooterMotorCurrentLimit = 50;
  public static final int feederMotorCurrentLimit = 30;

  public enum ShooterState {
    OFF(0.0, 0.0),
    SHOOTING(1.0, 1.0),
    FEEDING(1.0, 0.0);

    public final double shooterMotorSpeed;
    public final double feederMotorSpeed;

    ShooterState(double shooterMotorSpeed, double feederMotorSpeed) {
      this.shooterMotorSpeed = shooterMotorSpeed;
      this.feederMotorSpeed = feederMotorSpeed;
    }
  }
}
