package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterSystems.Hood;
import frc.robot.subsystems.ShooterSystems.Shooter;
import frc.robot.subsystems.ShooterSystems.ShotCalculator;
import frc.robot.subsystems.ShooterSystems.Turret;

public class AutoAim extends Command {
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Hood hood;
    private final Turret turret;
    private final ShotCalculator shotCalculator;

    private String lastTargetMode = "Hub";

    public AutoAim(
            Drivetrain drivetrain,
            Shooter shooter,
            Hood hood,
            Turret turret,
            ShotCalculator shotCalculator) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hood = hood;
        this.turret = turret;
        this.shotCalculator = shotCalculator;
        addRequirements(shooter, hood, turret, shotCalculator);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Optional<Alliance> alliance = DriverStation.getAlliance();

        boolean inScoringZone = FieldConstants.ShotZones.isInScoringZone(robotPose, alliance);
        boolean inPassingZone = FieldConstants.ShotZones.isInPassingZone(robotPose, alliance);
        boolean manualForcePassing = SmartDashboard.getBoolean("AutoAim/UsePassingTarget", false);
        boolean usePassingTarget = manualForcePassing || inPassingZone;

        Pose3d targetPose = usePassingTarget
                ? getBestPassingTargetForTurret(robotPose, alliance)
                : FieldConstants.Hub.getCenterForAllianceOrNearest(alliance, robotPose);
        lastTargetMode = usePassingTarget ? "Passing" : "Hub";

        shotCalculator.setTarget(targetPose);

        double distanceMeters = robotPose.getTranslation().getDistance(targetPose.toPose2d().getTranslation());
        double distanceInches = Units.metersToInches(distanceMeters);
        Constants.Shooter.ShotProfile profile = shotCalculator.getProfileForDistanceInches(distanceInches);
        if (profile == null) {
            return;
        }

        shooter.setSpeed(profile.speedRps());
        hood.setAngle(profile.hoodDeg());
        turret.setAngle(getTurretSetpointDegrees(robotPose, targetPose));

        SmartDashboard.putBoolean("AutoAim/InScoringZone", inScoringZone);
        SmartDashboard.putBoolean("AutoAim/InPassingZone", inPassingZone);
        SmartDashboard.putBoolean("AutoAim/PastAllianceHub", FieldConstants.ShotZones.isPastAllianceHub(robotPose, alliance));
        SmartDashboard.putBoolean("AutoAim/UsingPassingTarget", usePassingTarget);
        SmartDashboard.putBoolean("AutoAim/AllianceKnown", alliance.isPresent());
        SmartDashboard.putNumber("AutoAim/TargetX", targetPose.getX());
        SmartDashboard.putNumber("AutoAim/TargetY", targetPose.getY());
        SmartDashboard.putNumber("AutoAim/DistanceInches", distanceInches);
        SmartDashboard.putNumber("AutoAim/ShooterSetpointRps", profile.speedRps());
        SmartDashboard.putNumber("AutoAim/HoodSetpointDeg", profile.hoodDeg());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public String getLastTargetMode() {
        return lastTargetMode;
    }

    private double getTurretSetpointDegrees(Pose2d robotPose, Pose3d targetPose) {
        double turretRelativeDeg = getTurretRelativeDegrees(robotPose, targetPose);
        double minTurretDeg = Math.min(
                Constants.Turret.TurretPosition.LEFT.value,
                Constants.Turret.TurretPosition.RIGHT.value);
        double maxTurretDeg = Math.max(
                Constants.Turret.TurretPosition.LEFT.value,
                Constants.Turret.TurretPosition.RIGHT.value);
        return MathUtil.clamp(turretRelativeDeg, minTurretDeg, maxTurretDeg);
    }

    private double getTurretRelativeDegrees(Pose2d robotPose, Pose3d targetPose) {
        double targetYawFieldDeg = Math.toDegrees(
                Math.atan2(
                        targetPose.getY() - robotPose.getY(),
                        targetPose.getX() - robotPose.getX()));
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        return MathUtil.inputModulus(targetYawFieldDeg - robotHeadingDeg, -180.0, 180.0);
    }

    private boolean canTurretReachTarget(Pose2d robotPose, Pose3d targetPose) {
        double turretRelativeDeg = getTurretRelativeDegrees(robotPose, targetPose);
        double minTurretDeg = Math.min(
                Constants.Turret.TurretPosition.LEFT.value,
                Constants.Turret.TurretPosition.RIGHT.value);
        double maxTurretDeg = Math.max(
                Constants.Turret.TurretPosition.LEFT.value,
                Constants.Turret.TurretPosition.RIGHT.value);
        return turretRelativeDeg >= minTurretDeg && turretRelativeDeg <= maxTurretDeg;
    }

    private Pose3d getBestPassingTargetForTurret(Pose2d robotPose, Optional<Alliance> alliance) {
        Pose3d[] targets = alliance.isPresent()
                ? FieldConstants.Passing.getTargetsForAlliance(alliance)
                : new Pose3d[] {
                        FieldConstants.Passing.BLUE_LEFT_TARGET,
                        FieldConstants.Passing.BLUE_RIGHT_TARGET,
                        FieldConstants.Passing.RED_LEFT_TARGET,
                        FieldConstants.Passing.RED_RIGHT_TARGET
                };

        Pose3d closestOverall = targets[0];
        double closestOverallDist = robotPose.getTranslation().getDistance(closestOverall.toPose2d().getTranslation());

        Pose3d closestReachable = null;
        double closestReachableDist = Double.POSITIVE_INFINITY;

        for (Pose3d target : targets) {
            double dist = robotPose.getTranslation().getDistance(target.toPose2d().getTranslation());
            if (dist < closestOverallDist) {
                closestOverall = target;
                closestOverallDist = dist;
            }
            if (canTurretReachTarget(robotPose, target) && dist < closestReachableDist) {
                closestReachable = target;
                closestReachableDist = dist;
            }
        }

        SmartDashboard.putBoolean("AutoAim/PassingClosestReachable", closestReachable != null);
        return closestReachable != null ? closestReachable : closestOverall;
    }
}
