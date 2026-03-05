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
import static frc.robot.Constants.Shooter.DISTANCE_ANGLE_SPEED;
import static frc.robot.Constants.Shooter.ShotProfile;

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
        SmartDashboard.putBoolean("ShotCalc/UseVelocityComp", true);
        SmartDashboard.putBoolean("ShotCalc/UseAccelComp", true);
    }

    @Override
    public void periodic() {
        Pose2d drivetrainPose = drivetrain.getState().Pose;

        targetDistance = drivetrainPose.getTranslation().getDistance(targetLocation.toPose2d().getTranslation());
        if (!useManualTargetSpeed) {
            ShotProfile profile = getProfileForDistanceInches(
                    edu.wpi.first.math.util.Units.metersToInches(targetDistance));
            if (profile != null) {
                targetSpeedRps = profile.speedRps();
            }
        }

        Pose3d shooterPose = new Pose3d(drivetrainPose).plus(BALL_TRANSFORM_CENTER);

        ChassisSpeeds drivetrainSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                drivetrain.getState().Speeds, drivetrainPose.getRotation());
        boolean useVelocityComp = SmartDashboard.getBoolean("ShotCalc/UseVelocityComp", true);
        boolean useAccelComp = SmartDashboard.getBoolean("ShotCalc/UseAccelComp", true);
        if (!useVelocityComp) {
            drivetrainSpeeds = new ChassisSpeeds();
        }
        ChassisAccelerations drivetrainAccelerations = useAccelComp
                ? new ChassisAccelerations(drivetrainSpeeds, prevDrivetrainSpeeds, 0.02)
                : new ChassisAccelerations(new ChassisSpeeds(), new ChassisSpeeds(), 0.02);
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

    public ShotProfile getProfileForDistanceInches(double distanceInches) {
        if (DISTANCE_ANGLE_SPEED.isEmpty()) {
            return null;
        }
        var floor = DISTANCE_ANGLE_SPEED.floorEntry(distanceInches);
        var ceil = DISTANCE_ANGLE_SPEED.ceilingEntry(distanceInches);
        if (floor == null) {
            return ceil.getValue();
        }
        if (ceil == null) {
            return floor.getValue();
        }
        if (floor.getKey().equals(ceil.getKey())) {
            return floor.getValue();
        }

        double d0 = floor.getKey();
        double d1 = ceil.getKey();
        double t = (distanceInches - d0) / (d1 - d0);
        ShotProfile p0 = floor.getValue();
        ShotProfile p1 = ceil.getValue();
        double hoodDeg = p0.hoodDeg() + (p1.hoodDeg() - p0.hoodDeg()) * t;
        double speedRps = p0.speedRps() + (p1.speedRps() - p0.speedRps()) * t;
        return new ShotProfile(hoodDeg, speedRps);
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
