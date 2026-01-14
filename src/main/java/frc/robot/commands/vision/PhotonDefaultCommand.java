package frc.robot.commands.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;

public class PhotonDefaultCommand extends Command{
    private final PhotonVision vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout layout;
    private final Transform3d cameraToRobotOne;
    private final Transform3d cameraToRobotTwo;
    private final Transform3d cameraToRobotThree;
    private final Transform3d cameraToRobotFour;
    private final Transform3d cameraToRobotFive;

    private Pose2d combinedPose = new Pose2d();
    private Pose2d cachedRobotPose = new Pose2d();
    private Rotation2d finalrotation = new Rotation2d();

    private StructArrayLogEntry<Pose3d> visionTargetsLog;
    private StructArrayPublisher<Pose3d> visionTargetsPublisher;

    public PhotonDefaultCommand(PhotonVision camera, CommandSwerveDrivetrain drivetrain){
        this.vision = camera;
        this.drivetrain = drivetrain;
        this.layout = camera.getLayout();
        this.cameraToRobotOne = camera.getCamToRobotOne();
        this.cameraToRobotTwo = camera.getCamToRobotTwo();
        this.cameraToRobotThree = camera.getCamToRobotThree();
        this.cameraToRobotFour = camera.getCamToRobotFour();
        this.cameraToRobotFive = camera.getCamToRobotFive();
        addRequirements(camera);

        var log = DataLogManager.getLog();
        visionTargetsLog = StructArrayLogEntry.create(log, "/vision/targetPoses", Pose3d.struct);

        visionTargetsPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/vision/targetPoses", Pose3d.struct)
            .publish();
    }

    @Override
    public void execute(){
        PhotonPipelineResult PVresult = vision.getCamOneResult();
        PhotonPipelineResult PVresultTwo = vision.getCamTwoResult();
        PhotonPipelineResult PVresultThree = vision.getCamThreeResult();
        PhotonPipelineResult PVresultFour = vision.getCamFourResult();

        boolean hasTargetsOne = PVresult != null && PVresult.hasTargets();
        boolean hasTargetsTwo = PVresultTwo != null && PVresultTwo.hasTargets();
        boolean hasTargetsThree = PVresultThree != null && PVresultThree.hasTargets();
        boolean hasTargetsFour = PVresultFour != null && PVresultFour.hasTargets();

        SmartDashboard.putBoolean("pv one target", hasTargetsOne);
        SmartDashboard.putBoolean("pv two target", hasTargetsTwo);

        SmartDashboard.putBoolean("result null", PVresult != null);
        SmartDashboard.putBoolean("result null 2", PVresultTwo != null);

    

        

        if (PVresult != null && hasTargetsOne) {
            processSingleCam(PVresult, cameraToRobotOne);
        }
        if (PVresultTwo != null && hasTargetsTwo) {
            processSingleCam(PVresultTwo, cameraToRobotTwo);
        }
        if (PVresultThree != null && hasTargetsThree) {
            processSingleCam(PVresultThree, cameraToRobotThree);
        }
        if (PVresultFour != null && hasTargetsFour) {
            processSingleCam(PVresultFour, cameraToRobotFour);
        }

        logDetectedTags(PVresult, PVresultTwo, PVresultThree, PVresultFour);


    }

    public void processSingleCam(PhotonPipelineResult PVresult, Transform3d cameraToRobot){
        if (!PVresult.hasTargets()) {
            return;
        }

        PhotonTrackedTarget bestTarget = PVresult.getBestTarget();
        int tagId = bestTarget.getFiducialId();
        Optional<Pose3d> tagPoseOpt = layout.getTagPose(tagId);
        
        if(!tagPoseOpt.isPresent()){
            return;
        }
        
        Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(
            bestTarget.getBestCameraToTarget(), tagPoseOpt.get(), cameraToRobot);
        
        Pose2d robotPose = robotPose3d.toPose2d();
        cachedRobotPose = drivetrain.getState().Pose;

        double totalArea = getTotalTagArea(PVresult);

        SmartDashboard.putNumber("area", totalArea);

        double translationalVelocity = Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond);
        double rotationalVelocity = Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond);
        
        Matrix<N3, N1> stdDevs = calculateDynamicStdDevs(totalArea, translationalVelocity, rotationalVelocity);

        double poseDistance = cachedRobotPose.getTranslation().getDistance(robotPose.getTranslation());
        SmartDashboard.putNumber("distance", poseDistance);
        double rotationdiffrence = Math.abs(cachedRobotPose.getRotation().minus(robotPose.getRotation()).getDegrees());
        
        if(totalArea > LimelightConstants.kMinAreaOdom
        && totalArea < LimelightConstants.kMinAreaGyro 
        && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
        && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity
        && poseDistance < LimelightConstants.kMaxPoseDistance
        && rotationdiffrence < LimelightConstants.kMaxRotationDifference) {

            combinedPose = new Pose2d(
                robotPose.getX(), robotPose.getY(), cachedRobotPose.getRotation()
            ); 

            drivetrain.addVisionMeasurement(
                combinedPose,
                PVresult.getTimestampSeconds(),   
                stdDevs
            );
        }

        else if(totalArea > LimelightConstants.kMinAreaGyro
            && translationalVelocity < LimelightConstants.kMaxTranslationalVelocity
            && rotationalVelocity < LimelightConstants.kMaxRotationalVelocity
            && poseDistance < LimelightConstants.kMaxPoseDistance
            && rotationdiffrence < LimelightConstants.kMaxRotationDifference) {

            drivetrain.addVisionMeasurement(
                robotPose,
                PVresult.getTimestampSeconds(),
                stdDevs
            );
        }
    }

    

    private void logDetectedTags(PhotonPipelineResult resultOne, 
                                  PhotonPipelineResult resultTwo,
                                  PhotonPipelineResult resultThree,
                                  PhotonPipelineResult resultFour) {
        int totalTags = 0;
        
        if (resultOne != null && resultOne.hasTargets()) {
            totalTags += resultOne.getTargets().size();
        }
        if (resultTwo != null && resultTwo.hasTargets()) {
            totalTags += resultTwo.getTargets().size();
        }
        if (resultFour != null && resultFour.hasTargets()) {
            totalTags += resultFour.getTargets().size();
        }
      
        
        if (totalTags == 0) {
            visionTargetsLog.append(new Pose3d[0]);
            visionTargetsPublisher.set(new Pose3d[0]);
            return;
        }

        List<Pose3d> tagPoses = new ArrayList<>();

        // Add tags from camera one
        addCamTags(resultOne, tagPoses);
        addCamTags(resultTwo, tagPoses);
        addCamTags(resultThree, tagPoses);
        addCamTags(resultFour, tagPoses);
        
        Pose3d[] poseArray = tagPoses.toArray(new Pose3d[0]);

        visionTargetsLog.append(tagPoses);
        visionTargetsPublisher.set(poseArray);
    }

    public void addCamTags(PhotonPipelineResult r1, List<Pose3d> tagPoses) {
        if (r1 != null && r1.hasTargets()) {
            for (PhotonTrackedTarget target : r1.getTargets()) {
                int id = target.getFiducialId();
                Optional<Pose3d> tagPoseOptional = layout.getTagPose(id);
                tagPoses.add(tagPoseOptional.orElse(new Pose3d()));
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        visionTargetsPublisher.close();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    private double getTotalTagArea(PhotonPipelineResult result){
        if (!result.hasTargets()) {
            return 0.0;
        }
        double totalArea = 0.0;

        for (PhotonTrackedTarget target : result.getTargets()) {
            totalArea += target.getArea();
        }
        return totalArea;
    }

    private Matrix<N3, N1> calculateDynamicStdDevs(
        double totalArea, double translationalVelocity, double rotationalVelocity) {
        
        double basexyStdDev = LimelightConstants.kXyStdDev;
        double basethetaStdDev = LimelightConstants.kThetaStdDev;

        double areaFactor = 1.0 - (0.9 * Math.min(totalArea, 1.0));

        double transVelFactor = 0.3 + (1.5 * (translationalVelocity / LimelightConstants.kMaxTranslationalVelocity));

        double rotVelFactor = 0.3 + (1.5 * (rotationalVelocity / LimelightConstants.kMaxRotationalVelocity));

        double xyStdDev = basexyStdDev * areaFactor * transVelFactor;
        double thetaStdDev = basethetaStdDev * areaFactor * rotVelFactor;

        SmartDashboard.putNumber("Vision XY std", xyStdDev);
        SmartDashboard.putNumber("Vision Theta std", thetaStdDev);
        
        return edu.wpi.first.math.VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
}