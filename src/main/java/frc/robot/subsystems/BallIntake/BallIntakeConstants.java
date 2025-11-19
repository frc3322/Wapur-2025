package frc.robot.subsystems.BallIntake;

public class BallIntakeConstants{
    public static final int ballIntakeMotorCanId = 67;
    public static final int motorLimit = 40;





    public enum BallIntakeState{
        INTAKE(1.0),
        STOP(0.0);
    

        public final double motorSpeed; 

        BallIntakeState(double motorSpeed) {
            this.motorSpeed = motorSpeed;
        }
    }
}