package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase{
    
    private LED candle;

    private LimelightHelpers.PoseEstimate results;
    private LimelightHelpers.PoseEstimate resultsTwo;

    private Pose3d targetPoseOne;
    private Pose3d targetPoseTwo;

    private AprilTagFieldLayout layout;
    private int tags;


    public Limelight(LED candle){
        this.candle = candle;

    

        this.targetPoseOne = new Pose3d();
        this.targetPoseTwo = new Pose3d();

        this.tags = 0;

        try{
            this.layout = new AprilTagFieldLayout("/home/lvuser/deploy/2025-reefscape-welded.json");
        } catch(java.io.IOException e){
            this.layout = new AprilTagFieldLayout(java.util.List.of(), 0.0, 0.0);
            SmartDashboard.putString("LL layout error", e.getMessage());
        }
        LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightOne, LimelightConstants.kPipelineIndex);
        LimelightHelpers.setPipelineIndex(LimelightConstants.kLimelightTwo, LimelightConstants.kPipelineIndex);

    }

    @Override
    public void periodic(){
        results = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.kLimelightOne);
        resultsTwo = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.kLimelightTwo);


        SmartDashboard.putBoolean("Camera Connected", results != null);
        SmartDashboard.putBoolean("Camera Two Connected", resultsTwo != null);


        
        tags = 0;
        if (results != null) {
            tags += results.tagCount;
        }
        if (resultsTwo != null) {
            tags += resultsTwo.tagCount;
        }

        SmartDashboard.putNumber("tag count", tags);

       /*  if (hasValidTarget()){

            candle.cameraSetColor(Color.kGreen, 1);
        }
        else {
            candle.cameraClearColor();       

        } */
       
    }

    public LimelightHelpers.PoseEstimate getCamOneResult(){
        return results;
    }
    public LimelightHelpers.PoseEstimate getCamTwoResult(){
        return resultsTwo;
    }



    public AprilTagFieldLayout getLayout(){
        return layout;
    }

    public int getNumTags() {
        return tags;
    }
   
}