package frc.robot.SubSystem.FuelControl.Hopper;

import java.beans.Encoder;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SubSystem.Logging.NerdLog;

public class Hopper implements HopperIO{

    //for adjusting:
    
    double requestedVoltage;
    double requestedSpeed;

    //logging:
    double velocityRadPerSec;

    //functionals:
    SparkMax motor;
    RelativeEncoder encoder;
    BangBangController speedControl;

    public Hopper(SparkMax motor) {
        this.motor = motor;
        configureMotor(false);
        this.encoder = motor.getEncoder();
        speedControl = new BangBangController();
        
        velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());

        speedControl.setSetpoint(requestedSpeed);

        NerdLog.logDouble("Fuel Control/Hopper/ Requested Speed", requestedSpeed);
        NerdLog.logDouble("Fuel Control/Hopper/ Requested Voltage", requestedVoltage);
    }

    @Override
    public void hopperin() {
        motor.setVoltage(requestedVoltage * speedControl.calculate(velocityRadPerSec));
    }

    @Override
    public void hopperOut() {
        motor.set(-requestedVoltage * speedControl.calculate(-velocityRadPerSec));
    }

    @Override
    public void periodic() {

        velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());

        NerdLog.logDouble("Fuel Control/Hopper/ Hopper Velocity RadPerSec", Units.rotationsPerMinuteToRadiansPerSecond(velocityRadPerSec));


        requestedSpeed = NerdLog.getdouble("Fuel Management/ Hopper/ Requested Speed");
        requestedVoltage = NerdLog.getdouble("Fuel Management/ Hopper/ Requested Voltage");


        speedControl.setSetpoint(requestedSpeed);

    }

    private void configureMotor(boolean invert) {
        SparkMaxConfig config = new SparkMaxConfig();
         config.inverted(invert)
        .voltageCompensation(12)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20);
         config.signals.absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) 1000/100)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
