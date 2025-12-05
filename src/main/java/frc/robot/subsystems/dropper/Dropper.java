package frc.robot.subsystems.dropper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.dropper.DropperIO;
import frc.robot.subsystems.CrateIntake.CrateIntakeConstants.CrateIntakeState;
import frc.robot.subsystems.dropper.DropperConstants.DropperState;

public class Dropper extends SubsystemBase{
  private DropperIO dropperIO;
  private static Dropper instance;
  private final DropperIOInputsAutoLogged inputs = new DropperIOInputsAutoLogged();

  public DropperState dropperState = DropperState.STOP; 

  
  public Dropper(DropperIO dropperIO) { // The constructor (For med)
      this.dropperIO = dropperIO;
  }

  public static Dropper initialize(DropperIO dropperIO) { // Oh man do we love initialize
      if(instance == null){
          instance = new Dropper(dropperIO);
      }
      return instance;
    } 

    
    public void updateInputs(){
        DropperIO.updateInputs(inputs); //inputs be brakin unless buldin
        Logger.processInputs("Dropper", inputs);
    }


  @Override
  public void periodic() {
    updateInputs();
  }

   public void setState(DropperState State){
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