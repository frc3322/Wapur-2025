package frc.robot.subsystems.dropper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.dropper.DropperConstants.DropperState;
import org.littletonrobotics.junction.Logger;

public class Dropper extends SubsystemBase {
  private DropperIO dropperIO;
  private static Dropper instance;
  private final dropperIOInputsAutoLogged inputs = new dropperIOInputsAutoLogged();

  public DropperState dropperState = DropperState.STOP;

  public static Dropper initialize(DropperIOReal dropperIO) { // Oh man do we love initialize
    if (instance == null) {
      instance = new Dropper(dropperIO);
    }
    return instance;
  }

  public Dropper(DropperIO dropperIO) { // The constructor (For med)
    this.dropperIO = dropperIO;
  }

  public void updateInputs() {
    dropperIO.updateInputs(inputs); // inputs be brakin unless buldin
    Logger.processInputs("Dropper", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
  }

  public void setState(DropperState State) {
    setMotorsSpeed(State.motorSpeed);
  }

  public void setMotorsSpeed(double speed) {
    dropperIO.setMotorSpeeds(speed);
  }

  public Command goToStateCommand(DropperState State) {
    return new InstantCommand(() -> setState(State), this);
  }

  public Command setDropperStateCommand(DropperState state) {
    return new InstantCommand(() -> dropperState = state, this);
  }
}
