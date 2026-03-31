package frc.robot.SubSystem.Vision.AimAssist;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Num;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.SubSystem.Swerve.Drive;
import frc.robot.SubSystem.Vision.Vision;
import frc.robot.SubSystem.Vision.VisionIO;

public class AimAssist {
    VisionIO vision;
    double[] degreesRobotFront;
    double reccomendedHeading;
    double[] yawPerCam;
    
    ProfiledPIDController headingController = new ProfiledPIDController(0, 0, 0, 
        new TrapezoidProfile.Constraints(getreccomendedHeading(), getreccomendedHeading()));

        /**
         * 
         * @param vision
         * @param degreesRobotFront - the yaw in degrees that coordinates to the tag being directly in front of the robot
         */
    public AimAssist(VisionIO vision,  double[] degreesRobotFront) {
        this.vision = vision;
        this.degreesRobotFront = degreesRobotFront;
        yawPerCam = new double[degreesRobotFront.length];

    }

    /**
     * 
     * @return the heading that the vision wants to to turn to
     */
    public double getreccomendedHeading() {
        return reccomendedHeading;
    }

    public void periodic() {

        Optional<List<PhotonTrackedTarget>>[] targets =  vision.getTargets();
        for (int i  = 0; i< targets.length; i++) { // i = per camera selection
            if (targets[i].isEmpty()) {
                continue;
            }
            
            //iterate over cameras with basic checks
            for (PhotonTrackedTarget target : targets[i].get()) {

                //check for hub apriltags on the inside of the alliance zones, trying to score from neutral is a major foul
                if (target.getFiducialId() == 9 || 
                target.getFiducialId() == 10 ||
                target.getFiducialId() == 8 ||
                target.getFiducialId() == 5 ||
                target.getFiducialId() == 11||
                target.getFiducialId() == 2 ||
                target.getFiducialId() == 18 ||
                target.getFiducialId() == 27 ||
                target.getFiducialId() == 25 ||
                target.getFiducialId() == 26 ||
                target.getFiducialId() == 21 ||
                target.getFiducialId() == 24) {
                
                yawPerCam[i] = target.getYaw();
                if (target.getPoseAmbiguity() > 0.2) continue;

                   reccomendedHeading = degreesRobotFront[i] - target.getYaw();
                }
            }
        }
    }

    public double[] getYawsFromHub() {
        return yawPerCam;
    }
}
