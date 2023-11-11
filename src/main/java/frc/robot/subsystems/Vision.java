package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    PhotonCamera camera;
    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    
    public Vision(){
        camera = new PhotonCamera("photonvision");
    }

    public double getBestTargetError(){
        var result = camera.getLatestResult();
        double rotationSpeed = 0;
        if (result.hasTargets()) {
            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            // If we have no targets, stay still.
            rotationSpeed = 0;
        }
        return rotationSpeed;
    }
}
