package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterConstants.ShooterState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO shooterIO; 
    private final ShooterIOinputsAutoLogged inputs = new ShooterIOinputsAutoLogged;
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
        System.out.println("how to 67 on the rizz for ultimate sigmas with the greatest code ever found in ts");
    }
    
    public void setState(ShooterState State) {
        setMotorSpeed(State.motorspeed);
    }
    
}
