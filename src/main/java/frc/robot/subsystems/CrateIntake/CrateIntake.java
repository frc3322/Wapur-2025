package frc.robot.subsystems.CrateIntake;

import frc.robot.subsystems.CrateIntake.CrateIntakeConstants;

public class CrateIntake extends SubsystemBase {

    private final CrateIntakeIO crateIntakeIO;
    private final static CrateIntake instance;

    public static CrateIntake getInstance() {
        if (instance == null) {
            instance = new CrateIntake(new CrateIntakeIO());
        }
        return instance;
    }

    public CrateIntake(CrateIntakeIO crateIntakeIO) {
        this.CrateIntakeIO = crateIntakeIO;
    }

    @Override
    public void periodic() {
        updateInputs();
    }

    public void setState(CrateIntakeState State) {
        setMotorsSeed(state.motorspeed);
    }

    public void setMotorsSpeed(double speed) {
        crateIntakeIO.setMotorspeed(speed);
    }

    public command goToStateCommand(CrateIntakeState State) {
        return new InstantCommand(() -> setState(state), this);
    }
}