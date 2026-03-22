package frc.robot.SubSystem.FuelControl.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SubSystem.Logging.NerdLog;

public class Shooter implements ShooterIO {

    //for ShuffleBoard:
    ShuffleboardTab shuffleTab;
    double targetSpeedRadPerSec = 0.00;
    double RequestedVolts = 0.00;
    double bangCrtlTolerance = 0.00;


    //logging:
    double velocityRadPerSec;

    //functionals:
    BangBangController bangCrtl;
    SparkMax motor;
    RelativeEncoder encoder;

    /**
     * 
     * @param motor - take a guess
     */
    public Shooter(SparkMax motor) {
        this.motor = motor;
        configureMotor(false);
        this.encoder = motor.getEncoder();
        bangCrtl = new BangBangController();
        bangCrtl.setTolerance(bangCrtlTolerance);

        NerdLog.logDouble("Fuel Control/Shooter/ requestedVolts", RequestedVolts);
        NerdLog.logDouble("Fuel Control/Shooter/ target Speed Rad Per Sec", targetSpeedRadPerSec);
        NerdLog.logDouble("Fuel Control/Shooter/ bang control Tolerance", bangCrtlTolerance);
    }
    
    @Override
    public void shoot(boolean invert) {
        if (!invert) motor.setVoltage(RequestedVolts * bangCrtl.calculate(velocityRadPerSec));
        else motor.setVoltage(RequestedVolts * bangCrtl.calculate(velocityRadPerSec));
    }

    @Override
    public void stop() {
        motor.setVoltage(0.00);
    }

    @Override
    public boolean isShooting() {
        return bangCrtl.atSetpoint();//Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    }

    @Override
    public void periodic() {
        velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());

        NerdLog.logBooleanVariable("Fuel Control/Shooter/ Shooter is Shooting", isShooting());
        NerdLog.logBooleanVariable("Fuel Control/Shooter/ Shooter is Above set Speed", velocityRadPerSec > targetSpeedRadPerSec);
        NerdLog.logDouble("Fuel Control/Shooter/ Encoder Velocity RadPerSec", velocityRadPerSec);


        targetSpeedRadPerSec = NerdLog.getdouble("Fuel Control/Shooter/ requestedVolts");
        RequestedVolts = NerdLog.getdouble("Fuel Control/Shooter/ target Speed Rad Per Sec");
        bangCrtlTolerance = NerdLog.getdouble("Fuel Control/Shooter/ bang control Tolerance");

        bangCrtl.setTolerance(bangCrtlTolerance);
        bangCrtl.setSetpoint(targetSpeedRadPerSec);

    }

    //hopper uses same config
    private void configureMotor(boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
         config.inverted(inverted)
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
