package frc.robot;


import java.util.List;
import java.util.Optional;

import org.dyn4j.geometry.Transform;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.utils.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SubSystem.Climb.Climb;
import frc.robot.SubSystem.Climb.ClimbIO;
import frc.robot.SubSystem.FuelControl.FuelControl;
import frc.robot.SubSystem.Swerve.Drive;
import frc.robot.SubSystem.Vision.Vision;
import frc.robot.SubSystem.Vision.VisionIO;

public class AutoPicker {
    private static PIDController driveXPID = new PIDController(0, 0, 0);
    private static PIDController driveYPID = new PIDController(0, 0, 0);
    private static ProfiledPIDController driveThetaPID = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    private static double desiredVelocityMetersPerSec =1;
    private static HolonomicDriveController driveControls = new HolonomicDriveController(driveXPID, driveYPID, driveThetaPID);
    

    private static Drive drive;
    private static FuelControl fuelCrtl;
    private static ClimbIO climb;
    private static VisionIO vision;
    private static ChassisSpeeds DesiredChassisSpeeds;
    private static Transform3d[] robotToCamera; 

    /**
     *  do this is the robotContainer
     * @param drive
     * @param fuelCrtl
     * @param climb
     * @param vision
     */
    public static void SupplySubSystems(Drive drive, FuelControl fuelCrtl, ClimbIO climb, VisionIO vision, Transform3d[] robotToCamera) {
    }

    public static enum AutoRoutines {
        SHOOT_BALLS, CLIMB, SHOOT_BALLS_AND_CLIMB, SHOOT_BALLS_AND_COLLECT_DEPOSITE, SHOOT_BALLS_AND_GO_MIDDLE 
    }

    public static void PickAuto(AutoRoutines routine) {
        switch (routine) {
            case SHOOT_BALLS: ShootBalls(); break;
            case CLIMB: findClimbRack(); Climb(); break;
            case SHOOT_BALLS_AND_CLIMB:  break; // could use recursion, not lazy enough
            case SHOOT_BALLS_AND_COLLECT_DEPOSITE: break;
            case SHOOT_BALLS_AND_GO_MIDDLE: break;
        }

    }
    /**
     * for tuning the forward pid controller, forward 1 meter
     */
    public static void forwardTuningTest(Pose2d initialPose){
        DesiredChassisSpeeds = driveControls.calculate(initialPose, 
            new Pose2d(initialPose.getX() +1,initialPose.getY(),initialPose.getRotation()),
            desiredVelocityMetersPerSec, 
            new Rotation2d());
    }

    /**
     * for tuning sideways pid controller, left 1 meter
     */
    public static void sidewaysTuningTest(Pose2d initialPose) {
        DesiredChassisSpeeds = driveControls.calculate(initialPose, 
            new Pose2d(initialPose.getX(),initialPose.getY() + 1,initialPose.getRotation()),
            desiredVelocityMetersPerSec, 
            new Rotation2d());
    }

    /**
     *  for tuning rotation , does 180
     * @param initialPose
     */
    public static void rotationTuningTest(Pose2d initialPose) {
        DesiredChassisSpeeds = driveControls.calculate(initialPose, 
            new Pose2d(initialPose.getX(),initialPose.getY(),initialPose.getRotation().plus(new Rotation2d(Math.PI))), // does a 180
            desiredVelocityMetersPerSec, 
            new Rotation2d());
    }

    public static void defaultSpinBehavior() {
        boolean foundTarget = false;
        while (!foundTarget){
            for (Optional<List<PhotonTrackedTarget>> targetsPerCam : vision.getTargets()) {
                if (!targetsPerCam.isEmpty()) {
                    foundTarget = true; break;
                }
            }
        }
           
    }
    /** shot only */
    private static void ShootBalls() {
        
    }

    /** climb only 
     * MAKE SURE THE LIMITS ARE ON THE CLIMB FIRST!!!
    */
    private static void Climb() {

    }

    /**finds and moves the robot to the climb rack */
    private static void findClimbRack() {
        boolean AtClimb = false; //whether we are at the goal or not

        //this is going to be the holy mother of nesting
        do {
            //move closer to setpoint:
            Optional<Pose2d> desiredPose = Optional.empty();
            Optional<ChassisSpeeds> desiredChassis = Optional.empty();
            Optional<List<PhotonTrackedTarget>>[] targetsOptional = vision.getTargets();
            for (int i = 0; i < targetsOptional.length; i++) {
                if (targetsOptional[i].isEmpty()) continue;

                double maxAmbiguity = 1.1; // slightly heigher than pose ambig max from method
                for (PhotonTrackedTarget target : targetsOptional[i].get()) {
                    if (target.getPoseAmbiguity() > 0.2) continue; // we don't accept ambigous targets here

                    //filter for climb AprilTags
                    if ((target.getFiducialId() == 32 || target.getFiducialId() == 31 || target.getFiducialId() == 15 || target.getFiducialId() == 16)
                     && target.getPoseAmbiguity() < maxAmbiguity) {

                        maxAmbiguity = target.getPoseAmbiguity();
                        
                        //find targets global Pose:
                        double distancefromTarget = PhotonUtils.calculateDistanceToTargetMeters(robotToCamera[i].getZ(), // camera height
                            21.75, // of apriltags on climb rack per game manual
                            robotToCamera[i].getRotation().getY(),
                            target.getPitch());

                        double globalTargetAngleDeg = 180;//target.getFiducialId() == 15 || target.getFiducialId() == 16? 180 : 0; // gottabe either 180 || 0
                        double forwardDistance = distancefromTarget * Math.sin(target.getYaw());
                        double sidewaysDistance = distancefromTarget * Math.cos(target.getYaw());
                        Pose2d currentPose = drive.getEstimatedPose();
                        desiredPose = Optional.of(new Pose2d(currentPose.getX() + forwardDistance, 
                            sidewaysDistance + sidewaysDistance,
                             new Rotation2d(Units.degreesToRadians(globalTargetAngleDeg))));
                        
                        
                        
                    }
                }
            }

            if (!desiredPose.isEmpty()) {
                desiredChassis = Optional.of(driveControls.calculate(drive.getEstimatedPose(),
                 desiredPose.get(), desiredVelocityMetersPerSec,desiredPose.get().getRotation()));
            }

            if (desiredChassis.isEmpty()) desiredChassis = Optional.of(getDesiredChassisSpeeds()); // use previous chassisSpeed;
            else {
                DesiredChassisSpeeds = desiredChassis.get();
            }

            captureDesiredChassisSpeeds(desiredChassis.get());

            drive.move(DesiredChassisSpeeds.vxMetersPerSecond, DesiredChassisSpeeds.vyMetersPerSecond, DesiredChassisSpeeds.omegaRadiansPerSecond);

            AtClimb = driveControls.atReference();

            //check if we made it to setpoint
        } while (!AtClimb);

    }

    /** Shoot Balls, turns to climb rack, and does the whole climbing thing
     * MAKE SURE THE LIMITS ARE ON THE CLIMB FIRST!!!
    */
     private static void ShootAndClimb() {

    }

    private static void captureDesiredChassisSpeeds(ChassisSpeeds speeds) {
        DesiredChassisSpeeds = speeds;
    }

    private static ChassisSpeeds getDesiredChassisSpeeds() {
        return DesiredChassisSpeeds;
    }
}
