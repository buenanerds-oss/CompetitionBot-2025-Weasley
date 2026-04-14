package frc.robot;


import java.util.List;
import java.util.Optional;

import org.dyn4j.geometry.Transform;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
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
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SubSystem.Climb.Climb;
import frc.robot.SubSystem.Climb.ClimbIO;
import frc.robot.SubSystem.FuelControl.FuelControl;
import frc.robot.SubSystem.Swerve.Drive;
import frc.robot.SubSystem.Swerve.SwerveConstants;
import frc.robot.SubSystem.Vision.Vision;
import frc.robot.SubSystem.Vision.VisionIO;
import frc.robot.SubSystem.Vision.AimAssist.AimAssist;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;;

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
    private static AimAssist aimAssist;
    private static ChassisSpeeds DesiredChassisSpeeds;
    private static Transform3d[] robotToCamera; 

    /**
     *  do this is the robotContainer
     * @param drive
     * @param fuelCrtl
     * @param climb
     * @param vision
     */
    public static void supplySubSystems(Drive subdrive, FuelControl subfuelCrtl, ClimbIO subclimb, VisionIO subvision,AimAssist subaimAssist, Transform3d[] subrobotToCamera) {
        drive  = subdrive;
        fuelCrtl = subfuelCrtl;
        climb = subclimb;
        vision = subvision;
        aimAssist = subaimAssist;
        robotToCamera = subrobotToCamera;   
    }

    public static enum AutoRoutines {
        SHOOT_BALLS, SHOOT_BALLS_AND_CLIMB, RIGHT_OVER_BUMP, LEFT_OVER_BUMP, CRY
    }

    public static void pickAuto(AutoRoutines routine) {
        if (drive == null ||
        fuelCrtl == null ||
        climb == null ||
        vision == null ||
        aimAssist == null ||
        robotToCamera == null ||
        routine == null) return;
        switch (routine) {
            case SHOOT_BALLS: 
            drive.zeroOutModules();
                Timer backUpshoot = new Timer();
                backUpshoot.start();
                while (!backUpshoot.hasElapsed(1.25)) drive.move(-0.25, 0, 0);
                backUpshoot.stop();
                Timer shootTimerShoot = new Timer();
                shootTimerShoot.start();
                while (!shootTimerShoot.hasElapsed(8)) {
                    drive.move(0, 0, -aimAssist.getreccomendedHeading());
                    fuelCrtl.shootShooter();
                    fuelCrtl.outtake();
                    climb.climbDown();
                }
                shootTimerShoot.stop();

                break;


            case SHOOT_BALLS_AND_CLIMB:
                drive.zeroOutModules();
                Timer backUp = new Timer();
                backUp.start();
                while (!backUp.hasElapsed(2.25))  {
                    drive.move(-0.25, 0, 0); 
                    climb.climbDown();
                }
                backUp.stop();
                Timer shootTimer = new Timer();
                shootTimer.start();
                while (!shootTimer.hasElapsed(8) && DriverStation.isAutonomousEnabled()) {
                    drive.move(0, 0, 0);
                    fuelCrtl.shootShooter();
                    fuelCrtl.outtake();
                }
                shootTimer.stop();
                fuelCrtl.stopShooting();
                fuelCrtl.stopHopper();
                // distance from wall to hub - (distance from wall to tower  + (robtolength - camera distance from front))
                driveBackToTower(Units.inchesToMeters(120.75) - Units.inchesToMeters(19.5));
                drive.move(0, 0, 0);
                Timer climbUpTimer = new Timer();
                climbUpTimer.start();
                while (!climbUpTimer.hasElapsed(8) && DriverStation.isAutonomousEnabled()) climb.climbUp();
                climbUpTimer.stop();

                break;

                
                case RIGHT_OVER_BUMP:
                    drive.zeroOutModules();
                    Timer forwardTimer = new Timer();
                    forwardTimer.start();
                    while (!forwardTimer.hasElapsed(2)) drive.move(1, 0, 0);
                    Timer moveRight = new Timer();
                    moveRight.start();
                    while (!moveRight.hasElapsed(3)) drive.move(0.125, -0.25, 0);
                    drive.zeroOutModules();
                    Timer climbertimer = new Timer();
                    climbertimer.start();
                    while (!climbertimer.hasElapsed(5)) climb.climbDown();
                    climbertimer.stop();

                    break;

                case LEFT_OVER_BUMP :
                drive.zeroOutModules();
                Timer forwadsTimer = new Timer();
                forwadsTimer.start();
                while (!forwadsTimer.hasElapsed(2)) drive.move(1, 0, 0);
                Timer moveLeft = new Timer();
                moveLeft.start();
                while (!moveLeft.hasElapsed(3)) drive.move(0.125, 0.25, 0);
                moveLeft.stop();
                drive.zeroOutModules();
                Timer setClimber = new Timer();
                setClimber.start();
                while (setClimber.hasElapsed(5)) climb.climbDown();
                setClimber.stop();

                break;


                case CRY: return;


                default: return;
        }

    }

    /**
     * 
     * @param desiredDistanceFromHubMeters
     */
    private static void driveBackToTower(double desiredDistanceFromHubMeters) {
        //first get distance from hub using vision, then drive until at the desired distance from the hub:
        PhotonCamera intakeCam = RobotMap.Intakecam;
        PhotonTrackedTarget climbTarget = null;
        double distanceFromHubMeters = 0;
        double driveY = 0.00;
        while (distanceFromHubMeters < desiredDistanceFromHubMeters) {

            //get reccomened heading from intake cam: 11 in from front
            //16 && 32
            double reccomendedHeading = 0;
            List<PhotonPipelineResult> allresults = intakeCam.getAllUnreadResults();
            PhotonPipelineResult latestResult = allresults.get(allresults.size()-1);
            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                if ((target.getFiducialId() == 16 || target.getFiducialId() == 32) && target.getPoseAmbiguity() < 0.3) {
                    reccomendedHeading = Units.degreesToRadians(target.getYaw() - 5.00 ); // only use heading for the proper tag
                    climbTarget = target;
                }
            }


            Optional<List<PhotonTrackedTarget>>[] targets = vision.getTargets();
            for (int i = 0; i < targets.length; i++ ) {
                if (targets[i].isEmpty()) continue;

                double lowestAmbiguity = 0.75; // will base the distance off the cameras with the lowest ambiguity
                for (PhotonTrackedTarget target: targets[i].get()) { // per camera
                    if (target.getPoseAmbiguity() > lowestAmbiguity) continue;
                    lowestAmbiguity = target.getPoseAmbiguity();
                    if (target.getFiducialId() == 9 ||
                    target.getFiducialId() == 10 ||
                    target.getFiducialId() == 25 ||
                    target.getFiducialId() == 26) distanceFromHubMeters = PhotonUtils.calculateDistanceToTargetMeters(
                          robotToCamera[i].getZ(),
                          Units.inchesToMeters(44.25), 
                          robotToCamera[i].getRotation().getY(),
                          target.getPitch());
                    else if (climbTarget.getPoseAmbiguity()  < 0.35) { // i want this to be slightly more tolerable than normal
                      

                        distanceFromHubMeters = Units.inchesToMeters(158.6) - PhotonUtils.calculateDistanceToTargetMeters(
                                                 Units.inchesToMeters(21.5),
                                                 Units.inchesToMeters(21.75),
                                                 Units.degreesToRadians(0),
                                                 climbTarget.getPitch());
                    }
                }
            }

            //set drive proportional to the distance from the hub:
            double driveX = -(125 -distanceFromHubMeters)/Units.inchesToMeters(130); //scaled with the distance from hub to the wall
            drive.move(driveX, driveY, -reccomendedHeading);
        }
    }



    //everything after is deprecated but left for those after me to maybe learn from, if they can get a takeaway from it.
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
