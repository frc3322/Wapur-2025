package frc.robot.subsystems.CrateIntake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CrateIntake.CrateIntakeConstants.CrateIntakeState;
import org.littletonrobotics.junction.Logger;

public class CrateIntake extends SubsystemBase {

  private final CrateIntakeIO crateIntakeIO;
  private static CrateIntake instance;
  private final CrateIntakeIOInputsAutoLogged inputs = new CrateIntakeIOInputsAutoLogged();

  public static CrateIntake getInstance() {
    if (instance == null) {
      instance = new CrateIntake(new CrateIntakeIOReal());
    }
    return instance;
  }

  public CrateIntake(CrateIntakeIO crateIntakeIO) {
    this.crateIntakeIO = crateIntakeIO;
  }

  public void updateInputs() {
    CrateIntakeIO.updateInputs(inputs);
    Logger.processInputs("CrateIntake", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
  }

  public void setState(CrateIntakeState State) {
    setMotorsSpeed(State.motorSpeed);
  }

  public void setMotorsSpeed(double speed) {
    crateIntakeIO.setMotorspeed(speed);
  }

  public Command goToStateCommand(CrateIntakeState State) {
    return new InstantCommand(() -> setState(State), this);
  }
}
