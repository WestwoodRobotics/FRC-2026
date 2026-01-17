package frc.robot.commands.vision;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.FollowTrajectory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;

public class DriveToFuel extends Command {
    private final PhotonVision vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final Transform3d cameraToRobotFive;

    public DriveToFuel(PhotonVision vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.cameraToRobotFive = vision.getCamToRobotFive();
    }

    @Override
    public void initialize() {
        PhotonPipelineResult result = vision.getCamFiveResult();
        
        if (result == null || !result.hasTargets()) {
            cancel();
            return;
        }

        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d cameraToTarget = target.getBestCameraToTarget();

        if (cameraToTarget == null) {
            cancel();
            return;
        }

        Transform3d robotToTarget = cameraToRobotFive.plus(cameraToTarget);
        Pose2d robotPose = drivetrain.getState().Pose;

        Pose2d targetPose2d = robotPose.transformBy(
            new Transform2d(
                robotToTarget.getTranslation().toTranslation2d(),
                new Rotation2d()
            )
        );
        
        new FollowTrajectory(drivetrain, targetPose2d).schedule();
    }


     @Override
    public boolean isFinished() {
        return true; // End immediately after scheduling trajectory
    }

        

}
