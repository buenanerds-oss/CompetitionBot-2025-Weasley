package frc.robot.SubSystem.Vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

public class DriverCam {
    // camera for driver to see what their doing, does not need phtonVision

    public DriverCam(int ResloutionWidth, int ResolutionHeight,int FPS) {
        //make camera:
        UsbCamera Cam = CameraServer.startAutomaticCapture("IntakeCam", 0);

        //configure camera:
        Cam.setResolution(ResloutionWidth, ResolutionHeight); //TODO configure resolution
        Cam.setFPS(FPS); //todo configure FPS
    }

}
