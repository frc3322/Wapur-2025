package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterConstants.ShooterState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO shooterIO;
  private final ShooterIOinputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private static Shooter instance;

  private ShooterState currentState = ShooterState.STOP;

  public static Shooter Initalize() {
    if (instance == null) {
      instance = new Shooter(new ShooterIOReal());
    }
    return instance;
  }

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  public void updateInputs() {
    ShooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
  }

  public void setState(ShooterState state) {
    setMotorSpeed(state);
  }

  
}
