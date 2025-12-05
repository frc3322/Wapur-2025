package frc.robot.subsystems.dropper;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.dropper.DropperConstants;


public class DropperIOReal {
    
    public class DropperIOReal implements DropperIO {
        private final SparkMax dropperMotor; 


    public DropperIOReal() {
        dropperMotor = new SparkMax(DropperConstants.droppercanID, MotorType.kBrushless);
        

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        dropperMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(DropperConstants.motorLimit);

        dropperMotor.configure(dropperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void updateInputs(DropperIOInputsAutoLogged inputs) {
    inputs.dropperMotorSpeed = dropperMotor.getEncoder().getVelocity();
  }

  @Override
  public void setMotorspeed(double speed) {
    dropperMotor.set(speed);
  }
    }
}
