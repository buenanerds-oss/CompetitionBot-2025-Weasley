package frc.robot.SubSystem.Vision;

import java.time.chrono.ThaiBuddhistChronology;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.SubSystem.Logging.NerdLog;
import frc.robot.SubSystem.Swerve.Drive;

public class Vision implements VisionIO{
    //Adjustable:
    double maxAmbiguity = 0.3;

    //loggables:
    List<PhotonPipelineResult>[] allPhotonResults;
    PhotonPipelineResult[] latestResults;
        //ASSIGN THESE OPTIONALS BEFORE USE OR HELL WILL RAIN DOWN UPON YOU!!!
    Optional<EstimatedRobotPose>[] EstimatedPoses;
    Optional<Pose3d>[] confirmedPoses;
    Optional<List<PhotonTrackedTarget>>[] targets;

    //functionals:
    PhotonCamera[] cameras;
    Transform3d[] RobotToCameras;
    PhotonPoseEstimator[] estimators;
    Drive drive;

    /**
     * must enter all parameters in the same order as eachother or you'll get wonky results
     * @param cameras - the cameras you want to use
     * @param RobotToCameras - Where the camera is in relation to the robot
     */
    public Vision(PhotonCamera[] cameras, Transform3d[] RobotToCameras, Drive drive) {
        this.cameras = cameras;
        this.RobotToCameras = RobotToCameras;
        this.drive = drive;
        allPhotonResults = new List[cameras.length];
        estimators = new PhotonPoseEstimator[cameras.length];
        confirmedPoses = new Optional[cameras.length];
        targets = new Optional[0];

        for (int i = 0 ; i < estimators.length; i++ ) estimators[i] = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded), RobotToCameras[i]);
        periodic();
    }

    @Override
    public void periodic() {
        for (int i = 0 ; i < cameras.length ; i++) {
            
            //set all to empty because id rather have nothing than a bad output:
            targets[i] = Optional.empty();
            confirmedPoses[i] = Optional.empty();


            /*
             * evrything after this point will follow a get-then-check pattern:
             * this helps because almost nothing is garenteed
             */
            allPhotonResults[i] = cameras[i].getAllUnreadResults();

            if (allPhotonResults[i].isEmpty()) continue;
            latestResults[i] = allPhotonResults[i].get(allPhotonResults[i].size() -1);

            if (!latestResults[i].hasTargets()) continue;


            targets[i] = Optional.of(latestResults[i].getTargets());
            

            estimators[i].addHeadingData(Timer.getTimestamp(), drive.getEstimatedPose().getRotation());
            
            if (targets[i].get().size() > 1) EstimatedPoses[i] = estimators[i].estimateCoprocMultiTagPose(latestResults[i]);
            else if (targets[i].get().size() == 1) EstimatedPoses[i] = estimators[i].estimatePnpDistanceTrigSolvePose(latestResults[i]);


            //filtering the poses:

            boolean keepPose = true;

            if (targets[i].get().size() != 0) keepPose = false; // must have target
            else if (targets[i].get().size() == 1) {
                for (PhotonTrackedTarget target : targets[i].get()) {
                    if (target.getPoseAmbiguity() > maxAmbiguity) keepPose = false; // must be discernable
                }
            }

            if (EstimatedPoses[i].isEmpty()) keepPose = false;

                //must be in the field:
            if (EstimatedPoses[i].get().estimatedPose.getY() < 0 && EstimatedPoses[i].get().estimatedPose.getY() > 8.07) keepPose = false;

            if (keepPose) {
                NerdLog.logStructvariable("VisionPoses",EstimatedPoses[i].get().estimatedPose.toPose2d(), Pose2d.struct);
                drive.addvision(EstimatedPoses[i].get().estimatedPose.toPose2d(), latestResults[i].getTimestampSeconds()); // can also use Timer.getFPGATimestamp() if current doesn't work
            }
        }
    }

    @Override
    public Optional<List<PhotonTrackedTarget>>[] getTargets() {
        return targets;
    }

    @Override
    public Optional<Pose3d>[] getposeEstimations() {
        return confirmedPoses;
    }

    @Override
    public PhotonPipelineResult[] getLatestResults() {
        return latestResults;
    }
}
