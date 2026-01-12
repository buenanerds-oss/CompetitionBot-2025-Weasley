package frc.robot.SubSystem.Swerve;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ModuleMotorConfig {
    public static double TURN_P = 0.00;
    public static double TURN_I = 0.00;
    public static double TURN_D = 0.00;

    public static double DRIVE_P = 0.00;
    public static double DRIVE_I = 0.00;
    public static double DRIVE_D = 0.00;
    


       //all conversion factors convert to radians
        public static SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
        public static SparkMaxConfig turnMotorConfig = new SparkMaxConfig();

    public static void configureTurnMotor(SparkMax motor) { 
         //turn config
         turnMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)//FeedbackSensor.kPrimaryEncoder
         .p(TURN_P)
         .i(TURN_I)
         .d(TURN_D)
         .positionWrappingEnabled(true)
         .positionWrappingInputRange(0, 2*Math.PI);
         turnMotorConfig.inverted(true)
         .idleMode(IdleMode.kBrake)
         .smartCurrentLimit(20)
         .voltageCompensation(12.0);
         //signals
         turnMotorConfig.signals.primaryEncoderPositionAlwaysOn(true)
         .primaryEncoderPositionPeriodMs((int) 1000/100) // 100 = odometry frequency in hertz
         .primaryEncoderPositionAlwaysOn(true)
         .primaryEncoderVelocityPeriodMs(20)
         .appliedOutputPeriodMs(20)
         .busVoltagePeriodMs(20)
         .outputCurrentPeriodMs(20);
         turnMotorConfig.encoder.positionConversionFactor((2*Math.PI))
         .velocityConversionFactor((2*Math.PI)/60)
         .uvwMeasurementPeriod(10)
         .uvwAverageDepth(2);

         motor.configure(driveMotorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public static void configureDriveMotor(SparkMax motor) {
        driveMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(DRIVE_P)
        .i(DRIVE_I)
        .d(DRIVE_D);
        // normal config
        driveMotorConfig.idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
        .voltageCompensation(12.0);
        //signals
        driveMotorConfig.signals.primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) 1000/100) // 100 = odometry frequency in hertz
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
        //encoder
        driveMotorConfig.encoder.positionConversionFactor((2*Math.PI)/6.75) // 6.75 = conversion factor
        .velocityConversionFactor((2*Math.PI)/60/6.75)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

        motor.configure(driveMotorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    /**
     * changes the PID for the turn controller
     * @param valueToChange - P, I, D, only. no Variation allowed either
     * @param IncrementAmount - set negative to reduce value, set 0 for no changes
     */
    public static void changeTurnPID(String valueToChange, double IncrementAmount, SparkMax motor) {
        switch (valueToChange) {
            case "P":
                TURN_P += IncrementAmount;
                turnMotorConfig.closedLoop.p(TURN_P);
                break;
            case "I":
                TURN_I += IncrementAmount;
                turnMotorConfig.closedLoop.i(TURN_I);

                break;
            case "D":
                TURN_D += IncrementAmount;
                turnMotorConfig.closedLoop.d(TURN_D);

                break;
        
            default:
            return;
        }

        motor.configure(turnMotorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

    }

     /**
     * changes the PID for the drive controller
     * @param valueToChange - P, I, D, only. no Variation allowed either
     * @param IncrementAmount - set negative to reduce value
     */
    public static void changeDrivePID(String valueToChange, double IncrementAmount, SparkMax motor) {
        switch (valueToChange) {
            case "P":
                DRIVE_P += IncrementAmount;
                driveMotorConfig.closedLoop.p(TURN_P);
                break;
            case "I":
                DRIVE_I += IncrementAmount;
                driveMotorConfig.closedLoop.i(TURN_I);

                break;
            case "D":
                DRIVE_D += IncrementAmount;
                driveMotorConfig.closedLoop.d(TURN_D);

                break;
        
            default:
            return;
        }

        motor.configure(driveMotorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //simulated PID values, Thats all sim needs from this class:

    public static double SIM_DRIVE_P = 0.0;
    public static double SIM_DRIVE_I = 0.0;
    public static double SIM_DRIVE_D = 0.0;
    public static double SIM_TURN_P = 0.0;
    public static double SIM_TURN_I = 0.0;
    public static double SIM_TURN_D = 0.0;

}
