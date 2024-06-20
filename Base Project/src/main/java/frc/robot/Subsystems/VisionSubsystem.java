package frc.robot.Subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class VisionSubsystem extends SubsystemBase {

    private static VisionSubsystem INSTANCE;
    PhotonCamera camera;

    private VisionSubsystem() {
        camera = new PhotonCamera("test");
    }

    public static VisionSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VisionSubsystem();
        }
        return INSTANCE;
    }

    public List<PhotonTrackedTarget> getTrackedTargets() {
        return getRawResults().targets;
    }

    public PhotonPipelineResult getRawResults() {
        return camera.getLatestResult();
    }
}

