package frc.robot.SubSystem.Swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.RobotMap;
import frc.robot.SubSystem.Logging.GroupLogger;
import frc.robot.SubSystem.Logging.NerdLog;
import frc.robot.Util.BasicUtil;


public class Module extends ModuleMotorConfig implements ModuleIO {

    
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
    SparkClosedLoopController turnSparkPID;
    SparkClosedLoopController driveSparkPID;
    boolean atSetpoint = false;

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

        this.turnSparkPID = turnMotor.getClosedLoopController();
        this.driveSparkPID = driveMotor.getClosedLoopController();
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

        turnPID.enableContinuousInput(0, 2* Math.PI);
        turnPID.setTolerance(0.2);
        
    }
    @Override
    public void setDesiredSwerveState(SwerveModuleState desiredState) {
        //desiredState.optimize(new Rotation2d(currentSwerveState.angle.getRadians()));// ensures the turning setpoint takes the most effecient route
        desiredState.cosineScale(new Rotation2d(currentSwerveState.angle.getRadians()));// prevents undesired rotation or translation
        this.desiredState = desiredState;

        
        //turning
            turnPID.setSetpoint(desiredState.angle.getRadians());
            turnMotor.setVoltage(turnPID.calculate(currentSwerveState.angle.getRadians()));
          
        //Driving
            //manual cosine scaling because the abs encoder doesnt want to work for the scaling:
            desiredState.speedMetersPerSecond *= Math.cos(turnPID.getError()); 
      
            driveMotor.setVoltage(-desiredState.speedMetersPerSecond*5); // was *5
        

        //record changes:
        turnAmps = turnMotor.getOutputCurrent();
            turnVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
            driveAmps =  driveMotor.getOutputCurrent();
            driveVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        currentSwerveState = new SwerveModuleState(driveMotor.getEncoder().getVelocity() * SwerveConstants.wheelRadiusMeters, 
        new Rotation2d(AbsEncoder.get()));
        currentswervePosition = new SwerveModulePosition(driveMotor.getEncoder().getPosition() * SwerveConstants.wheelRadiusMeters, currentSwerveState.angle);
        
    }

    @Override
    public void forceSetVoltage(double turnVolts, double driveVolts) {
        driveMotor.setVoltage(driveVolts);
        turnMotor.setVoltage(turnVolts);
    }

    @Override
    public void periodic() {
        GroupLogger.logDoubleGroup("abs Encode readings", AbsEncoder.get(), index, 4);
        GroupLogger.logStructGroup("Drive/Modules/Swerve Module States", currentSwerveState, SwerveModuleState.struct, index, 4);
        GroupLogger.logDoubleGroup("Drive/Modules/Swerve Module Turn Amps", turnAmps, index, 4);
        GroupLogger.logDoubleGroup("Drive/Modules/Swerve Module Turn volts", turnVolts, index, 4);
        GroupLogger.logDoubleGroup("Drive/Modules/Swerve Module drive Amps", driveAmps, index, 4);
        GroupLogger.logDoubleGroup("Drive/Modules/Swerve Module drive Volts", driveVolts, index, 4);
        GroupLogger.logStructGroup("Drive/Modules/Swerve Module States", currentSwerveState, SwerveModuleState.struct, index, 4);
        GroupLogger.logStructGroup("Drive/Modules/swerve module Positions", currentswervePosition, SwerveModulePosition.struct, index, 4);
        GroupLogger.LogBooleanGroup("Drive/Modules/Module Turn @ setpoint", turnPID.atSetpoint(), index, 4);
        GroupLogger.logStructGroup("Drive/Modules/desired state", desiredState, SwerveModuleState.struct, index, 4);
        GroupLogger.logDoubleGroup("Drive/Modules/Turn Error", turnPID.getError(), index, 4);

        
    }

    @Override
    public SwerveModulePosition getmodulePosition() {
        return currentswervePosition;
         
    }
    


    

    

}
