package frc.robot.subsystems.Dropper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CrateIntake.CrateIntakeIO;
import frc.robot.subsystems.Dropper.DropperConstants.DropperState;

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
        Logger.processInputs("CrateIntake", inputs);
    }


  @Override
  public void periodic() {
    updateInputs();
  }

  public void setState(DropperState State){
    setMotorsSpeed(State.motorSpeed);
  }

  public void setMotorsSpeed(double speed) {
    DropperIO.setMotorsSpeed(speed);
  }

}
