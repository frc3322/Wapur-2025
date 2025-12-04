package frc.robot.subsystems.Shooter;

public class ShooterConstants {
  public static final int ShooterCanId = 9;
  public static final int FeederCanId = 14;
  public static final int ShooterMotorLimit = 40;
  public static final int FeederMotorLimit = 40;

  public enum ShooterState {
    SHOOTING(1.0, 1.0),
    FEEDING(1.0, 0.0),
    STOP(0.0, 0.0);

    public final double shooterMotorSpeed;
    public final double feederMotorSpeed;

    ShooterState(double shooterMotorSpeed, double feederMotorSpeed) {
      this.feederMotorSpeed = feederMotorSpeed;
      this.shooterMotorSpeed = shooterMotorSpeed;
    }
  }
}
