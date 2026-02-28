package frc.robot.subsystems.ShooterSystems;

import com.techhounds.houndutil.houndlib.ChassisAccelerations;
import com.techhounds.houndutil.houndlib.ShootOnTheFlyCalculator;
import com.techhounds.houndutil.houndlib.ShootOnTheFlyCalculator.InterceptSolution;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.techhounds.houndutil.houndlog.annotations.Tunable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.Shooter.BALL_TRANSFORM_CENTER;
import static frc.robot.Constants.Shooter.DISTANCE_TO_SHOT_SPEED;

// stores current target and actively computes effective target
@LoggedObject
public class ShotCalculator extends SubsystemBase {
    private final Drivetrain drivetrain;
    private ChassisSpeeds prevDrivetrainSpeeds = new ChassisSpeeds();

    @Log
    private Pose3d currentEffectiveTargetPose = Pose3d.kZero;

    @Log
    private double currentEffectiveYaw;

    private InterceptSolution currentInterceptSolution;

    @Log
    private Pose3d targetLocation = FieldConstants.Hub.CENTER;

    @Log
    private double targetDistance = 0.0;

    @Log
    @Tunable
    private double targetSpeedRps = 8;
    private boolean useManualTargetSpeed = false;

    public ShotCalculator(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        Pose2d drivetrainPose = drivetrain.getState().Pose;

        targetDistance = drivetrainPose.getTranslation().getDistance(targetLocation.toPose2d().getTranslation());
        if (!useManualTargetSpeed) {
            targetSpeedRps = DISTANCE_TO_SHOT_SPEED.get(targetDistance);
        }

        Pose3d shooterPose = new Pose3d(drivetrainPose).plus(BALL_TRANSFORM_CENTER);

        ChassisSpeeds drivetrainSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds, drivetrainPose.getRotation());
        ChassisAccelerations drivetrainAccelerations = new ChassisAccelerations(
                drivetrainSpeeds, prevDrivetrainSpeeds, 0.02);
        prevDrivetrainSpeeds = drivetrainSpeeds;

        try {
            currentInterceptSolution = ShootOnTheFlyCalculator.solveShootOnTheFly(shooterPose, targetLocation,
                    drivetrainSpeeds, drivetrainAccelerations, targetSpeedRps,
                    5, 0.01);
        } catch (IllegalArgumentException ex) {
            currentInterceptSolution = new InterceptSolution(Pose3d.kZero, 0.0, 0.0, 0.0, 0.0);
        }

        currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose();
        currentEffectiveYaw = currentInterceptSolution.requiredYaw();
    }

    public void setTarget(Pose3d targetLocation) {
        this.targetLocation = targetLocation;
        this.useManualTargetSpeed = false;
    }

    public void setTarget(Pose3d targetLocation, double targetSpeedRps) {
        this.targetLocation = targetLocation;
        this.targetSpeedRps = targetSpeedRps;
        this.useManualTargetSpeed = true;
    }

    public Pose3d getCurrentEffectiveTargetPose() {
        return currentEffectiveTargetPose;
    }

    public double getCurrentEffectiveYaw() {
        return currentEffectiveYaw;
    }

    public InterceptSolution getInterceptSolution() {
        return currentInterceptSolution;
    }

    public double getTargetSpeedRps() {
        return targetSpeedRps;
    }
    public void printDiagnostics() {
        SmartDashboard.putNumber("Shot Calculator Target Distance", targetDistance);
        SmartDashboard.putNumber("Shot Calculator Target Speed RPS", targetSpeedRps);
        SmartDashboard.putNumber("Shot Calculator Effective Target X", currentEffectiveTargetPose.getX());
        SmartDashboard.putNumber("Shot Calculator Effective Target Y", currentEffectiveTargetPose.getY());
        SmartDashboard.putNumber("Shot Calculator Effective Target Z", currentEffectiveTargetPose.getZ());
        SmartDashboard.putNumber("Shot Calculator Effective Yaw", currentEffectiveYaw);
    }
}
