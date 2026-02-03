package frc.robot.SubSystem.Swerve;

import javax.lang.model.type.PrimitiveType;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotMap;
import frc.robot.SubSystem.Logging.GroupLogger;
import frc.robot.SubSystem.Logging.NerdLog;
import frc.robot.Util.BasicUtil;


// TODO sparkmax encoder, not representing its movement proeperly, likely tied to position disconnect
public class Module extends ModuleMotorConfig implements ModuleIO {

    
    boolean EmergencyStop = false;
    double turnAmps;
    double turnVolts;
    double driveAmps;
    double driveVolts;
    SwerveModuleState currentSwerveState;
    SwerveModulePosition currentswervePosition;
    SwerveModuleState desiredState;

    double turnLocation;
    int index;
    SparkMax turnMotor;
    SparkMax driveMotor;
    PIDController drivePID;
    PIDController turnPID;
    AnalogEncoder AbsEncoder;

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
        this.AbsEncoder = RobotMap.swerveAbsEncoders[index];


        //from the ModuleMotorConfig Class
        configureTurnMotor(turnMotor);
        configureDriveMotor(driveMotor);
       // this.turnController = turnMotor.getClosedLoopController();
        //this.driveController = driveMotor.getClosedLoopController();
        this.currentSwerveState = new SwerveModuleState(driveMotor.getEncoder().getVelocity() * SwerveConstants.wheelRadiusMeters, //SpeedRadPS * RadiusMeters = velocityMetersPerSecond
         new Rotation2d(turnMotor.getEncoder().getPosition()));
        this.currentswervePosition = new SwerveModulePosition(driveMotor.getEncoder().getPosition() * SwerveConstants.wheelRadiusMeters, new Rotation2d(turnMotor.getEncoder().getPosition()));

        //prevents initialization issues
        turnAmps = 0;
        turnVolts = 0;
        driveAmps = 0; 
        driveVolts = 0;
        desiredState = new SwerveModuleState();

        //PID initialization:
        drivePID = new PIDController(ModuleMotorConfig.DRIVE_P, ModuleMotorConfig.DRIVE_I, ModuleMotorConfig.DRIVE_D);
        turnPID = new PIDController(ModuleMotorConfig.TURN_P, ModuleMotorConfig.TURN_I, ModuleMotorConfig.TURN_D);
    }
    @Override
    public void setDesiredSwerveState(SwerveModuleState desiredState) {
        desiredState.optimize(currentSwerveState.angle); // modules doesn;t rotate more than 180DEG most of the time
        desiredState.cosineScale(currentSwerveState.angle); // Smoother driving
        this.desiredState = desiredState;

        //Force Stops everything if we're asking for more resources than what we are supposed to
        if (turnMotor.getOutputCurrent() >= SwerveConstants.turnMaxAmps || driveMotor.getOutputCurrent() >= SwerveConstants.driveMaxAmps || // current
         turnMotor.getAppliedOutput() * turnMotor.getBusVoltage() >= SwerveConstants.turnMaxVolts || driveMotor.getAppliedOutput() * driveMotor.getBusVoltage() >= SwerveConstants.driveMaxVolts//voltage
        ) EmergencyStop = true;

        //turning
        //if we are not at the position AND we aren't in emergencyStop, keep running PID.
        if (!BasicUtil.numIsInBallparkOf(currentSwerveState.angle.getRadians(), desiredState.angle.getRadians(), SwerveConstants.turnAccuracyToleranceRAD) && !EmergencyStop) {
           turnPID.setSetpoint(desiredState.angle.getRadians());
           turnMotor.setVoltage(turnPID.calculate(AbsEncoder.get()));
        }
        else {turnPID.reset();
            turnMotor.setVoltage(0);} // if we don't need it to move, stop giving it the voltage to move


        /*
        //drive
        //the equation in the getter of the velocity converts from RPM to RADPM to MPS
        if (!BasicUtil.numIsInBallparkOf(currentSwerveState.speedMetersPerSecond, desiredState.speedMetersPerSecond, SwerveConstants.driveAccuracyToleranceMPS) && !EmergencyStop && !BasicUtil.numIsInBallparkOf(desiredState.speedMetersPerSecond, 0, SwerveConstants.driveAccuracyToleranceMPS)) {

        }
        else if (!EmergencyStop && !BasicUtil.numIsInBallparkOf(desiredState.speedMetersPerSecond, 0, SwerveConstants.driveAccuracyToleranceMPS)) {
            driveMotor.setVoltage(driveVolts); // if it works, keep doing what your doing. no need to run PID for longer than it needs
        }
        else driveMotor.setVoltage(0); //hard stop it in emergency mode

        */

        //record changes:
        turnAmps = turnMotor.getOutputCurrent();
            turnVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
            driveAmps =  driveMotor.getOutputCurrent();
            driveVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        currentSwerveState = new SwerveModuleState(driveMotor.getEncoder().getVelocity() * SwerveConstants.wheelRadiusMeters, 
        new Rotation2d(AbsEncoder.get()));
        currentswervePosition = new SwerveModulePosition(driveMotor.getEncoder().getPosition() * SwerveConstants.wheelRadiusMeters, new Rotation2d(AbsEncoder.get()));
        
    }

    @Override
    public void forceSetVoltage(double turnVolts, double driveVolts) {
        driveMotor.setVoltage(driveVolts);
        turnMotor.setVoltage(turnVolts);
    }

    @Override
    public void changeModuleTurnPID(String valueToChange, double IncrementAmount) {
        forceSetVoltage(0, 0);
        changeTurnPID(valueToChange, IncrementAmount);
        turnPID = new PIDController(ModuleMotorConfig.TURN_P,
         ModuleMotorConfig.TURN_I,
         ModuleMotorConfig.TURN_D);
    }

    @Override
    public void changeModuleDrivePID(String valueToChange, double IncrementAmount) {
        forceSetVoltage(0, 0);
        changeDrivePID(valueToChange, IncrementAmount);
        drivePID = new PIDController(ModuleMotorConfig.DRIVE_P,
         ModuleMotorConfig.DRIVE_I,
         ModuleMotorConfig.DRIVE_D);
    }

    @Override
    public void periodic() {
        GroupLogger.logStructGroup("Swerve Module States", currentSwerveState, SwerveModuleState.struct, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module Turn Amps", turnAmps, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module Turn volts", turnVolts, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module drive Amps", driveAmps, index, 4);
        GroupLogger.logDoubleGroup("Swerve Module drive Volts", driveVolts, index, 4);
        GroupLogger.LogBooleanGroup("Emergency Stopped?", EmergencyStop, index, 4);
        GroupLogger.logStructGroup("Swerve Module States", currentSwerveState, SwerveModuleState.struct, index, 4);
        GroupLogger.logStructGroup("swerve module Positions", currentswervePosition, SwerveModulePosition.struct, index, 4);
        GroupLogger.LogBooleanGroup("Module Turn @ setpoint", turnPID.atSetpoint(), index, 4);
        GroupLogger.logStructGroup("desired state", desiredState, SwerveModuleState.struct, index, 4);
        if (RobotState.isDisabled()) EmergencyStop = false;
    }

    @Override
    public SwerveModulePosition getmodulePosition() {
        return currentswervePosition;
         
    }
    


    

    

}
