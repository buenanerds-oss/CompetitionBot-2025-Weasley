package frc.robot.SubSystem.Climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.SubSystem.Logging.NerdLog;

public class Climb implements ClimbIO{

    double requestedVolts = 12;
    double upLimitRAD = 0.00; // limit for climb when climbing Up
    double downLimitRAD = 0.00; // limit for climbing when climbing down

    //functionals:
    SparkMax motor;
    RelativeEncoder encoder;

    public Climb(SparkMax motor) {
        this.motor = motor;
        configureMotor(false);
        this.encoder = motor.getEncoder();
    }

    @Override
    public void climbUp() {
        motor.setVoltage(requestedVolts);
    }

    @Override
    public void climbDown() {
        motor.setVoltage(-requestedVolts);
    }

    @Override
    public boolean atLimit(boolean checkUpLimit) {
        return checkUpLimit? encoder.getPosition() > upLimitRAD : encoder.getPosition() < downLimitRAD;
    }

    @Override
    public void periodic() {
        NerdLog.logDouble("Climb/ Position", encoder.getPosition() * (2 * Math.PI)); // records in radians
        NerdLog.logBooleanVariable("Climb/ @ Up limit", encoder.getPosition() * (2*Math.PI) >= upLimitRAD);
        NerdLog.logBooleanVariable("Climb/ @ Down limit", encoder.getPosition() * (2*Math.PI) <= downLimitRAD);
    }

    //TODO: configure your darn motor
    private void configureMotor(boolean inverted) {
         SparkMaxConfig config = new SparkMaxConfig();

        
        config.inverted(inverted)
        .voltageCompensation(12)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);
         config.signals.absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) 1000/100)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
        //config.encoder.positionConversionFactor(gearbox);

        //LimitSwitch
        //config.limitSwitch.forwardLimitSwitchPosition(forwardLimit);
        //config.limitSwitch.reverseLimitSwitchPosition(reverselimit);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
}
