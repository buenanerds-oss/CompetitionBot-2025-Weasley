package frc.robot.SubSystem.FuelControl.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.SubSystem.Logging.NerdLog;

public class Shooter implements ShooterIO {

    //for ShuffleBoard:
    ShuffleboardTab shuffleTab;
    double targetSpeedRadPerSec = 400;
    double RequestedVolts = 8.45;
    double CrtlTolerance = 50;
    Timer autoTime;


    //logging:
    double velocityRadPerSec;

    //functionals:
    PIDController pidCrtl = new PIDController(0.5, 0, 0); // did not use, but still theire for alternative
    BangBangController bangCrtl;
    SparkMax motor;
    RelativeEncoder encoder;

    /**
     * 
     * @param motor - take a guess
     */
    public Shooter(SparkMax motor) {
        this.motor = motor;
        configureMotor(true);
        this.encoder = motor.getEncoder();
        bangCrtl = new BangBangController();
        bangCrtl.setTolerance(CrtlTolerance);
        autoTime = new Timer();

        NerdLog.logDouble("Fuel Control/Shooter/ requestedVolts", RequestedVolts);
        NerdLog.logDouble("Fuel Control/Shooter/ target Speed Rad Per Sec", targetSpeedRadPerSec);
        NerdLog.logDouble("Fuel Control/Shooter/ control Tolerance", CrtlTolerance);
        /* 
        NerdLog.logDouble("Fuel Control/Shooter/ P", pidCrtl.getP());
        NerdLog.logDouble("Fuel Control/Shooter/ I", pidCrtl.getI());
        NerdLog.logDouble("Fuel Control/Shooter/ D", pidCrtl.getD());
        */
        
    }
    
    @Override
    public void shoot(boolean invert) {
        NerdLog.logBooleanVariable("is trying to shoot in shooter", true);
        if (!invert) {

            NerdLog.logDouble("Fuel Control/Shooter/planned Output", 8.45 * bangCrtl.calculate(velocityRadPerSec));
            //bang bang control:
            bangCrtl.setSetpoint(targetSpeedRadPerSec);
            motor.setVoltage(RequestedVolts * bangCrtl.calculate(velocityRadPerSec));
            NerdLog.logDouble("Fuel Control/Shooter/ bang crtl output", bangCrtl.calculate(velocityRadPerSec));
            

            /*
            // pid control:
            pidCrtl.setSetpoint(targetSpeedRadPerSec);
            RequestedVolts += pidCrtl.calculate(velocityRadPerSec);
            motor.setVoltage(RequestedVolts);
            */
        }

        else {

            
            //bang bang control:
            bangCrtl.setSetpoint(targetSpeedRadPerSec);
            motor.setVoltage(-RequestedVolts * bangCrtl.calculate(-velocityRadPerSec));
            

            /*
            pid control:
            pidCrtl.setSetpoint(-targetSpeedRadPerSec);
            RequestedVolts += pidCrtl.calculate(velocityRadPerSec);
            motor.setVoltage(RequestedVolts);
            */            
        }
    }

    @Override
    public void stop() {
        RequestedVolts = 0.00;
        motor.setVoltage(0.00);
    }

    @Override
    public boolean isShooting() {
        if (DriverStation.isAutonomous()) {
            RequestedVolts = 9.5;
           autoTime.start();
           if (autoTime.hasElapsed(5)) {
            autoTime.stop();
            return true;
           }
        }
        RequestedVolts = 8.45;
        return bangCrtl.atSetpoint();//velocityRadPerSec > targetSpeedRadPerSec - bangCrtlTolerance;//bangCrtl.atSetpoint();//Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
    }

    @Override
    public void periodic() {
        velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());

        NerdLog.logBooleanVariable("Fuel Control/Shooter/ Shooter is Shooting", isShooting());
        NerdLog.logBooleanVariable("Fuel Control/Shooter/ Shooter is Above set Speed", velocityRadPerSec > targetSpeedRadPerSec);
        NerdLog.logDouble("Fuel Control/Shooter/ Encoder Velocity RadPerSec", velocityRadPerSec);
        NerdLog.logDouble("Fuel Control//Shooter/ applied voltage", motor.getAppliedOutput() * motor.getBusVoltage());
        NerdLog.logDouble("Fuel Control//Shooter/ bus voltage", motor.getBusVoltage());


        //RequestedVolts = NerdLog.getdouble("Fuel Control/Shooter/ requestedVolts");
       targetSpeedRadPerSec= NerdLog.getdouble("Fuel Control/Shooter/ target Speed Rad Per Sec");
        CrtlTolerance = NerdLog.getdouble("Fuel Control/Shooter/ control Tolerance");
        /*
        pidCrtl.setP(NerdLog.getdouble("Fuel Control/Shooter/ P"));
        pidCrtl.setI(NerdLog.getdouble("Fuel Control/Shooter/ I"));
        pidCrtl.setD(NerdLog.getdouble("Fuel Control/Shooter/ D"));
        */

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
