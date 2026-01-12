package frc.robot.SubSystem.Swerve;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.SubSystem.Logging.GroupLogger;
import frc.robot.Util.BasicUtil;

public class Module extends ModuleMotorConfig implements ModuleIO {

    
    boolean EmergencyStop = false;
    double turnAmps;
    double turnVolts;
    double driveAmps;
    double driveVolts;
    SwerveModuleState currentSwerveState;

    int index;
    SparkMax turnMotor;
    SparkMax driveMotor;
    SparkClosedLoopController turnController;
    SparkClosedLoopController driveController;

    /**
     * one indivual SwerveModule, 4 of these makes the swerve drive. starts count @ 0
     * @param index - which module is this?
     * @param turnMotor - turn encoder
     * @param driveMotor - drive encoder
     */
    public Module(int index, SparkMax turnMotor, SparkMax driveMotor) {
        this.index = index;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;

        //from the ModuleMotorConfig Class
        configureTurnMotor(turnMotor);
        configureDriveMotor(driveMotor);
        this.turnController = turnMotor.getClosedLoopController();
        this.driveController = driveMotor.getClosedLoopController();
        this.currentSwerveState = new SwerveModuleState(Units.rotationsPerMinuteToRadiansPerSecond(driveMotor.getEncoder().getVelocity()) * SwerveConstants.wheelRadiusMeters, //SpeedRadPS * RadiusMeters = velocityMetersPerSecond
         new Rotation2d(Units.rotationsToRadians(turnMotor.getEncoder().getPosition())));

        //prevents initialization issues
        turnAmps = 0;
        turnVolts = 0;
        driveAmps = 0; 
        driveVolts = 0;
    }

    @Override
    public void setDesiredSwerveState(SwerveModuleState desiredState) {
        desiredState.optimize(currentSwerveState.angle); // modules doesn;t rotate more than 180DEG most of the time
        desiredState.cosineScale(currentSwerveState.angle); // Smoother driving

        //Force Stops everything if we're asking for more resources than what we are supposed to
        if (turnMotor.getOutputCurrent() >= SwerveConstants.turnMaxAmps || driveMotor.getOutputCurrent() >= SwerveConstants.driveMaxAmps || // current
         turnMotor.getAppliedOutput() * turnMotor.getBusVoltage() >= SwerveConstants.turnMaxVolts || driveMotor.getAppliedOutput() * driveMotor.getBusVoltage() >= SwerveConstants.driveMaxVolts//voltage
        ) EmergencyStop = true;

        //turning
        //if we are not at the position AND we aren't in emergencyStop, keep running PID.
        if (!BasicUtil.numIsInBallparkOf(currentSwerveState.angle.getRadians(), desiredState.angle.getRadians(), SwerveConstants.turnAccuracyToleranceRAD) && !EmergencyStop) {
            turnController.setReference(Units.radiansToRotations(desiredState.angle.getRadians()), ControlType.kPosition);
        }
        else turnMotor.setVoltage(0); // if we don't need it to move, stop giving it the voltage to move


        
        //drive
        //the equation in the getter of the velocity converts from RPM to RADPM to MPS
        if (!BasicUtil.numIsInBallparkOf(currentSwerveState.speedMetersPerSecond, desiredState.speedMetersPerSecond, SwerveConstants.driveAccuracyToleranceMPS) && !EmergencyStop && !BasicUtil.numIsInBallparkOf(desiredState.speedMetersPerSecond, 0, SwerveConstants.driveAccuracyToleranceMPS)) {

        }
        else if (!EmergencyStop && !BasicUtil.numIsInBallparkOf(desiredState.speedMetersPerSecond, 0, SwerveConstants.driveAccuracyToleranceMPS)) {
            driveMotor.setVoltage(driveVolts); // if it works, keep doing what your doing. no need to run PID for longer than it needs
        }
        else driveMotor.setVoltage(0); //hard stop it in emergency mode

        //record changes

        turnAmps = turnMotor.getOutputCurrent();
        turnVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        driveAmps =  driveMotor.getOutputCurrent();
        driveVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        currentSwerveState = new SwerveModuleState(Units.rotationsPerMinuteToRadiansPerSecond(driveMotor.getEncoder().getVelocity()) * SwerveConstants.wheelRadiusMeters, 
        new Rotation2d(Units.rotationsToRadians(turnMotor.getEncoder().getPosition())));
        
    }

    @Override
    public void forceSetVoltage(double turnVolts, double driveVolts) {
        driveMotor.setVoltage(driveVolts);
        turnMotor.setVoltage(turnVolts);
    }

    @Override
    public void changeModuleTurnPID(String valueToChange, double IncrementAmount) {
        forceSetVoltage(0, 0);
        changeTurnPID(valueToChange, IncrementAmount, turnMotor);
        EmergencyStop = false;
    }

    @Override
    public void changeModuleDrivePID(String valueToChange, double IncrementAmount) {
        forceSetVoltage(0, 0);
        changeDrivePID(valueToChange, IncrementAmount, driveMotor);
        EmergencyStop = false;
    }

    @Override
    public void periodic() {
        GroupLogger.logStructGroup("Swerve Module States", currentSwerveState, SwerveModuleState.struct, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module Turn Amps", turnAmps, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module Turn volts", turnVolts, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module Turn Amps", driveAmps, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module Turn Amps", driveVolts, index, 4);

    }

    @Override
    public SwerveModulePosition getmodulePosition() {
        return new SwerveModulePosition(Units.rotationsToRadians(driveMotor.getEncoder().getPosition()) * SwerveConstants.wheelRadiusMeters,
         new Rotation2d(Units.rotationsToRadians(turnMotor.getEncoder().getPosition())));
         
    }
    


    

    

}
