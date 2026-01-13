package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import pabeles.concurrency.IntOperatorTask.Max;

public class AutoLock extends Command{
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentricFacingAngle faceCenter;
    private final DoubleSupplier orbitDoubleSupplier;
    private final DoubleSupplier radiusDoubleSupplier;
    private double MaxAngularRate;
    private double MaxSpeed;
    private Optional<Alliance> ally = DriverStation.getAlliance();
    double dx;
    double dy;

    public AutoLock(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentricFacingAngle faceCenter, DoubleSupplier orbitDoubleSupplier, DoubleSupplier radiusDoubleSupplier, double maxAngleRate, double maxSpeed){
        this.drivetrain = drivetrain;
        this.faceCenter = faceCenter;
        this.orbitDoubleSupplier = orbitDoubleSupplier;
        this.radiusDoubleSupplier = radiusDoubleSupplier;
        this.MaxAngularRate = maxAngleRate;
        this.MaxSpeed = maxSpeed;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        if(ally.isPresent()){
            Pose2d currentPose = drivetrain.getState().Pose;
            SmartDashboard.putNumber("pose", currentPose.getX());
            SmartDashboard.putNumber("posey", currentPose.getY());


                if(ally.get() == Alliance.Red){
                    double dx = TrajectoryConstants.kRedHub.getX() - currentPose.getX();
                    double dy = TrajectoryConstants.kRedHub.getY() - currentPose.getY();
                }

                if(ally.get() == Alliance.Blue){
                    double dx = TrajectoryConstants.kBlueHub.getX() - currentPose.getX();
                    double dy = TrajectoryConstants.kBlueHub.getY() - currentPose.getY();
                }

    
            Rotation2d targetAngle = new Rotation2d(Math.atan2(dy, dx) + Math.PI);
            double radius = Math.hypot(dx, dy);
            
            double input = orbitDoubleSupplier.getAsDouble();
            double radiusinput = radiusDoubleSupplier.getAsDouble();

            double tangentUx = -dy / radius;
            double tangentUy = dx / radius; 

            double radiusUx = dx / radius;
            double radiusUy = dy / radius;
            
            double angularRate = MaxAngularRate * input;
            double tangentialSpeed = angularRate * radius;
            

            double radiusSpeed = radiusinput * MaxSpeed;
        
            double vx = tangentUx*tangentialSpeed + radiusUx*radiusSpeed;
            double vy = tangentUy*tangentialSpeed + radiusUy*radiusSpeed;
            double speed = Math.hypot(vx, vy);

            if(speed > MaxSpeed){
                double scale = MaxSpeed / speed;
                vx *= scale;
                vy *= scale;
            }

            SmartDashboard.putNumber("Target angle degrees", targetAngle.getDegrees());
            SmartDashboard.putNumber("Angle error degrees", targetAngle.minus(currentPose.getRotation()).getDegrees());
            if(ally.get() == Alliance.Red && currentPose.getX() < TrajectoryConstants.kRedHub.getX()
             || ally.get() == Alliance.Blue && currentPose.getX() > TrajectoryConstants.kBlueHub.getX()){
            drivetrain.setControl(faceCenter
            .withTargetDirection(targetAngle)
            .withVelocityX(vx)
            .withVelocityY(vy)
            );
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}