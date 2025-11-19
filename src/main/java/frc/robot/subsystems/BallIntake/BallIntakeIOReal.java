package frc.robot.subsystems.BallIntake;

import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.CrateIntake.CrateIntakeIO;

public class BallIntakeIOReal implements CrateIntakeIO {
    private final SparkMax motor;
    
    public BallIntakeIOReal() {
        motor = new SparkMax(0, null);
    }
}
