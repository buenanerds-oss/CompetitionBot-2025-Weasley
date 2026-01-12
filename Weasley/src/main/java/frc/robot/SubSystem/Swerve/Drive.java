package frc.robot.SubSystem.Swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.SubSystem.Logging.NerdLog;
import frc.robot.SubSystem.Swerve.Gyro.GyroIO;

public class Drive implements Subsystem {
    
    SwerveDriveKinematics swerveKino = new SwerveDriveKinematics(SwerveConstants.moduletranslations);
    ModuleIO[] modules;
    GyroIO gyro;
    SwerveDrivePoseEstimator poseEstimator;
    ChassisSpeeds currentChassis;

    /**
     * the thing that controls the driving
     * @param gyro - the Gyro your using
     * @param modules - modules in the order of FL, FR, BL, BR
     */
    public Drive(GyroIO gyro, ModuleIO... modules) {
        this.modules = modules;
        this.gyro = gyro;
        poseEstimator = new SwerveDrivePoseEstimator(swerveKino, new Rotation2d(gyro.getAngleRad()), new SwerveModulePosition[4], SwerveConstants.initialPose);
        currentChassis = new ChassisSpeeds();


    }

    /**
     * move the robot
     * 
     * @param x - forward if positive
     * @param y - left if positive
     * @param rot - rotates counterclockwise If positive, must be Radians
     */
    public void move(double x, double y, double rot) {
        ChassisSpeeds DesiredChassis = new ChassisSpeeds(x, y, rot); // what we want
        DesiredChassis = ChassisSpeeds.fromFieldRelativeSpeeds(DesiredChassis, new Rotation2d(gyro.getAngleRad())); //from where we see it at a fixed point to the way the robot is facing which can be variable.
        DesiredChassis = ChassisSpeeds.discretize(DesiredChassis, SwerveConstants.timeStepToMove); // tells it to only  move for as long as it needs to

        SwerveModuleState[] desiredStates = swerveKino.toSwerveModuleStates(DesiredChassis);

        for (int i = 0; i < modules.length; i++) modules[i].setDesiredSwerveState(desiredStates[i]);

        //updating stuff after changes

        currentChassis = DesiredChassis;

        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < modulePositions.length;i++ ) modulePositions[i] = modules[i].getmodulePosition();
        poseEstimator.update(new Rotation2d(gyro.getAngleRad()), modulePositions);
    }

    @Override
    public void periodic() {
        //logs in periodic
        NerdLog.logStructvariable("Current Chassis", currentChassis, ChassisSpeeds.struct);
        NerdLog.logStructvariable("DrivePose", poseEstimator.getEstimatedPosition(), Pose2d.struct);

        NerdLog.logDouble(" Module Drive PID P", Robot.isReal()? ModuleMotorConfig.DRIVE_P : ModuleMotorConfig.SIM_DRIVE_P);
        NerdLog.logDouble(" Module Drive PID I", Robot.isReal()? ModuleMotorConfig.DRIVE_I : ModuleMotorConfig.SIM_DRIVE_I);
        NerdLog.logDouble(" Module Drive PID D", Robot.isReal()? ModuleMotorConfig.DRIVE_D : ModuleMotorConfig.SIM_DRIVE_D);
        NerdLog.logDouble(" Module turn PID P",  Robot.isReal()? ModuleMotorConfig.TURN_P : ModuleMotorConfig.SIM_TURN_P);
        NerdLog.logDouble(" Module Turn PID I",  Robot.isReal()? ModuleMotorConfig.TURN_I : ModuleMotorConfig.SIM_TURN_I);
        NerdLog.logDouble(" Module Turn PID D",  Robot.isReal()? ModuleMotorConfig.TURN_D : ModuleMotorConfig.SIM_TURN_D);
        
        for (ModuleIO module : modules) module.periodic();
    }
}
