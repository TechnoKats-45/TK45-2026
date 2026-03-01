package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private final Drivetrain drivetrain;
    private final PhotonCamera leftCamera;
    private final PhotonCamera rightCamera;
    private final PhotonPoseEstimator leftEstimator;
    private final PhotonPoseEstimator rightEstimator;
    private int leftFuseCount = 0;
    private int rightFuseCount = 0;

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        leftCamera = new PhotonCamera(Constants.Vision.LEFT_CAMERA_NAME);
        rightCamera = new PhotonCamera(Constants.Vision.RIGHT_CAMERA_NAME);

        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        leftEstimator = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.ROBOT_TO_LEFT_CAMERA);
        leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        rightEstimator = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                Constants.Vision.ROBOT_TO_RIGHT_CAMERA);
        rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        processCamera(leftCamera, leftEstimator, "Left");
        processCamera(rightCamera, rightEstimator, "Right");
    }

    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator, String label) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        SmartDashboard.putNumber("Vision/" + label + "/UnreadResults", results.size());

        if (results.isEmpty()) {
            SmartDashboard.putBoolean("Vision/" + label + "/HasTarget", false);
            SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
            return;
        }

        for (PhotonPipelineResult result : results) {
            SmartDashboard.putBoolean("Vision/" + label + "/HasTarget", result.hasTargets());

            if (!result.hasTargets()) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                continue;
            }

            if (result.getBestTarget() != null) {
                SmartDashboard.putNumber("Vision/" + label + "/BestTagId", result.getBestTarget().getFiducialId());
                SmartDashboard.putNumber("Vision/" + label + "/BestTagAmbiguity", result.getBestTarget().getPoseAmbiguity());
            }

            Optional<EstimatedRobotPose> estimate = estimator.update(result);
            if (estimate.isEmpty()) {
                SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", false);
                continue;
            }

            Pose2d estimatedPose = estimate.get().estimatedPose.toPose2d();
            Matrix<N3, N1> stdDevs = getVisionStdDevs(result);
            drivetrain.addVisionMeasurement(estimatedPose, estimate.get().timestampSeconds, stdDevs);
            SmartDashboard.putBoolean("Vision/" + label + "/EstimateAccepted", true);

            SmartDashboard.putNumber("Vision/" + label + "/TagCount", result.targets.size());
            SmartDashboard.putNumber("Vision/" + label + "/X", estimatedPose.getX());
            SmartDashboard.putNumber("Vision/" + label + "/Y", estimatedPose.getY());
            SmartDashboard.putNumber("Vision/" + label + "/ThetaDeg", estimatedPose.getRotation().getDegrees());
            SmartDashboard.putNumber("Vision/" + label + "/TimestampSec", estimate.get().timestampSeconds);
            if ("Left".equals(label)) {
                leftFuseCount++;
                SmartDashboard.putNumber("Vision/Left/FuseCount", leftFuseCount);
            } else {
                rightFuseCount++;
                SmartDashboard.putNumber("Vision/Right/FuseCount", rightFuseCount);
            }
        }
    }

    private Matrix<N3, N1> getVisionStdDevs(PhotonPipelineResult result) {
        int tagCount = result.targets.size();
        // Multi-tag solves are typically much more trustworthy than single-tag solves.
        if (tagCount >= 2) {
            return VecBuilder.fill(0.4, 0.4, 0.8);
        }
        return VecBuilder.fill(1.0, 1.0, 2.0);
    }
}
