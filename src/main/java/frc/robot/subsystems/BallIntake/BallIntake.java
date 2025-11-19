package frc.robot.subsystems.BallIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.BallIntake.BallIntakeConstants.BallIntakeState;

public class BallIntake extends SubsystemBase {

    private final BallIntakeIO ballIntakeIO;
    private static BallIntake instance;

    private final BallIntakeIOInputsAutologged inputs = new BallIntakeIOInputsAutologged();

    
    public static BallIntake getInstance() {
        if (instance == null) {
                instance = new BallIntake(new BallIntakeIOReal());
            }
        return instance;
    }

    public BallIntake(BallIntakeIO ballIntakeIO) {
        this.ballIntakeIO = ballIntakeIO;
    }

    public void updateInputs() {
        BallIntakeIO.updateInputs(inputs);
        Logger.processInputs("BallIntake", inputs);

    }
    
    @Override
    public void periodic() {
        updateInputs();
    }


    public void setState(BallIntakeState State) {
        ballIntakeIO.setMotorspeed(State.motorSpeed); 
    }

    public void setMotorspeed(double speed) {
        ballIntakeIO.setMotorspeed(speed);
    }

    public Command goToStateCommand(BallIntakeState state) {
        return new InstantCommand(() -> setState(state), this);
    }
}

        
        

