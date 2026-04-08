package frc.robot.SubSystem.Vision.AimAssist;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.PhotonPipelineResultProto;

public class IntakeAssist {
    /*
     * follows under the concept that the camera can read objects and put them similar to 
     * apriltags in the system
     */

     PhotonCamera camera;
     public IntakeAssist(PhotonCamera camera) {
        this.camera = camera;
     }

     public double getReccomendedHeading() {
        List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
        if (unreadResults.isEmpty()) return 0;

        PhotonPipelineResult latestResult = unreadResults.get(unreadResults.size()-1);
        PhotonTrackedTarget bestTarget = latestResult.getBestTarget();

        
        return bestTarget.getYaw();
     }
}
