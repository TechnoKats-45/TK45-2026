package frc.robot.commands;

import java.util.Locale;
import java.util.Optional;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Spindex;
import frc.robot.subsystems.ShooterSystems.Hood;
import frc.robot.subsystems.ShooterSystems.Shooter;
import frc.robot.subsystems.ShooterSystems.ShotCalculator;
import frc.robot.subsystems.ShooterSystems.Turret;
import com.techhounds.houndutil.houndlib.ShootOnTheFlyCalculator.InterceptSolution;

public class AutoShoot extends Command {
    private enum AutoShootMode {
        OFF,
        PASS,
        SCORE
    }

    // All SmartDashboard tune keys are under this prefix.
    // Change values live while enabled to tune behavior without redeploy.
    private static final String DASH_PREFIX = "AutoShoot/";
    private static final NetworkTable FMS_TABLE = NetworkTableInstance.getDefault().getTable("FMSInfo");
    private String allianceSource = "DriverStation";

    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Hood hood;
    private final Turret turret;
    private final Spindex spindex;
    private final BallElevator ballElevator;
    private final ShotCalculator shotCalculator;
    private final BooleanSupplier autoEnabledSupplier;
    private final BooleanSupplier scoreEnableSupplier;
    private final Field2d zoneField = new Field2d();
    private final StructArrayPublisher<Pose3d> trajectory3dRawPublisher = NetworkTableInstance.getDefault()
            .getTable("AutoShoot")
            .getStructArrayTopic("Trajectory3dRaw", Pose3d.struct)
            .publish();
    private final StructArrayPublisher<Pose3d> trajectory3dAllowedPublisher = NetworkTableInstance.getDefault()
            .getTable("AutoShoot")
            .getStructArrayTopic("Trajectory3dAllowed", Pose3d.struct)
            .publish();

    public AutoShoot(
            Drivetrain drivetrain,
            Shooter shooter,
            Hood hood,
            Turret turret,
            Spindex spindex,
            BallElevator ballElevator,
            ShotCalculator shotCalculator,
            BooleanSupplier autoEnabledSupplier,
            BooleanSupplier scoreEnableSupplier) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hood = hood;
        this.turret = turret;
        this.spindex = spindex;
        this.ballElevator = ballElevator;
        this.shotCalculator = shotCalculator;
        this.autoEnabledSupplier = autoEnabledSupplier;
        this.scoreEnableSupplier = scoreEnableSupplier;

        addRequirements(shooter, hood, turret, spindex, ballElevator, shotCalculator);
        putDashboardDefaults();
        SmartDashboard.putData("AutoShoot Zones", zoneField);
        // Seed the field overlay so zone lines are visible before robot is enabled.
        publishZoneDebug(new Pose2d(), AutoShootMode.OFF);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        AutoShootMode mode = determineMode(robotPose);
        publishZoneDebug(robotPose, mode);

        if (mode == AutoShootMode.OFF) {
            stopFeedAndShooter();
            clearShotTrajectory();
            SmartDashboard.putString(DASH_PREFIX + "Mode", mode.name());
            SmartDashboard.putBoolean(DASH_PREFIX + "CanFeedNow", false);
            return;
        }

        Alliance currentAlliance = getAllianceForAutoShoot();
        Pose3d allianceHubCenter = FieldConstants.Hub.getCenterForAlliance(Optional.of(currentAlliance));
        Pose3d target = (mode == AutoShootMode.SCORE) ? allianceHubCenter : getBestPassTarget(robotPose, allianceHubCenter);
        if (mode == AutoShootMode.SCORE) {
            shotCalculator.setTarget(target);
        } else {
            shotCalculator.setTarget(target, SmartDashboard.getNumber(DASH_PREFIX + "PassShotSpeedRps", 11.0));
        }

        Pose3d shooterPose = new Pose3d(robotPose);
        Pose3d effectiveTarget = shotCalculator.getCurrentEffectiveTargetPose();
        if (effectiveTarget.equals(Pose3d.kZero)) {
            effectiveTarget = target;
        }

        double dx = effectiveTarget.getX() - shooterPose.getX();
        double dy = effectiveTarget.getY() - shooterPose.getY();
        double fieldYawRad = Math.atan2(dy, dx);
        double robotRelativeYawDeg = Math.toDegrees(MathUtil.angleModulus(fieldYawRad - robotPose.getRotation().getRadians()));

        turret.setAngle(robotRelativeYawDeg);
        InterceptSolution intercept = shotCalculator.getInterceptSolution();
        if (intercept == null) {
            intercept = new InterceptSolution(target, 0.0, 0.0, 0.0, 0.0);
        }
        TrajectoryData trajectoryData = buildShotTrajectory(shooterPose, effectiveTarget, intercept);
        publishRawTrajectory(trajectoryData);

        hood.setAngle(Math.toDegrees(intercept.launchPitchRad()));
        shooter.setTargetSpeedRps(shotCalculator.getTargetSpeedRps());

        boolean canScoreWindow = mode != AutoShootMode.SCORE || canScoreHubNow();
        boolean hasBallisticSolution = intercept.flightTime() > 0.0
                && intercept.launchSpeed() > 0.0;
        boolean inRange = robotPose.getTranslation().getDistance(target.toPose2d().getTranslation())
                <= SmartDashboard.getNumber(
                        DASH_PREFIX + (mode == AutoShootMode.SCORE ? "ScoreMaxDistanceM" : "PassMaxDistanceM"),
                        mode == AutoShootMode.SCORE ? 6.0 : 10.0);

        double aimToleranceDeg = SmartDashboard.getNumber(
                DASH_PREFIX + (mode == AutoShootMode.SCORE ? "ScoreAimToleranceDeg" : "PassAimToleranceDeg"),
                mode == AutoShootMode.SCORE ? 2.5 : 6.0);
        double speedToleranceRps = SmartDashboard.getNumber(
                DASH_PREFIX + (mode == AutoShootMode.SCORE ? "ScoreSpeedToleranceRps" : "PassSpeedToleranceRps"),
                mode == AutoShootMode.SCORE ? 1.0 : 2.5);

        boolean readyToFeed = canScoreWindow
                && hasBallisticSolution
                && inRange
                && shooter.isAtSpeed(speedToleranceRps)
                && hood.isAligned(aimToleranceDeg)
                && turret.isAligned(aimToleranceDeg);
        publishAllowedTrajectory(trajectoryData, canScoreWindow);

        if (readyToFeed) {
            double feedPercent = SmartDashboard.getNumber(
                    DASH_PREFIX + (mode == AutoShootMode.SCORE ? "ScoreFeedPercent" : "PassFeedPercent"),
                    mode == AutoShootMode.SCORE ? 0.85 : 1.0);
            spindex.runFeed(feedPercent);
            ballElevator.runFeed(feedPercent);
        } else {
            spindex.stop();
            ballElevator.stop();
        }

        SmartDashboard.putString(DASH_PREFIX + "Mode", mode.name());
        SmartDashboard.putBoolean(DASH_PREFIX + "CanFeedNow", readyToFeed);
        SmartDashboard.putBoolean(DASH_PREFIX + "CanScoreWindow", canScoreWindow);
        SmartDashboard.putNumber(DASH_PREFIX + "RobotRelativeYawDeg", robotRelativeYawDeg);
    }

    @Override
    public void end(boolean interrupted) {
        stopFeedAndShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private AutoShootMode determineMode(Pose2d robotPose) {
        if (!autoEnabledSupplier.getAsBoolean()) {
            return AutoShootMode.OFF;
        }

        double x = robotPose.getX();
        ZoneBoundaries boundaries = getZoneBoundaries();
        if (x > boundaries.blueScoreMaxX && x < boundaries.redScoreMinX) {
            return AutoShootMode.PASS;
        }

        Alliance alliance = getAllianceForAutoShoot();

        boolean onScoringSide = (alliance == Alliance.Blue)
                ? x <= boundaries.blueScoreMaxX
                : x >= boundaries.redScoreMinX;
        if (onScoringSide) {
            // On scoring side, SCORE is operator gated (LT in RobotContainer).
            // If not held, this returns OFF and robot will not shoot.
            return scoreEnableSupplier.getAsBoolean() ? AutoShootMode.SCORE : AutoShootMode.OFF;
        }

        return AutoShootMode.PASS;
    }

    private void publishZoneDebug(Pose2d robotPose, AutoShootMode mode) {
        ZoneBoundaries boundaries = getZoneBoundaries();
        double passMinX = boundaries.blueScoreMaxX;
        double passMaxX = boundaries.redScoreMinX;
        double blueScoreMaxX = boundaries.blueScoreMaxX;
        double redScoreMinX = boundaries.redScoreMinX;

        Alliance alliance = getAllianceForAutoShoot();
        boolean isRed = alliance == Alliance.Red;
        double scoreBoundaryX = isRed ? redScoreMinX : blueScoreMaxX;

        SmartDashboard.putString(DASH_PREFIX + "ZoneNow", mode.name());
        SmartDashboard.putNumber(DASH_PREFIX + "RobotX_m", robotPose.getX());
        SmartDashboard.putNumber(DASH_PREFIX + "RobotX_in", Units.metersToInches(robotPose.getX()));

        SmartDashboard.putNumber(DASH_PREFIX + "PassBandMinX_m", passMinX);
        SmartDashboard.putNumber(DASH_PREFIX + "PassBandMaxX_m", passMaxX);
        SmartDashboard.putNumber(DASH_PREFIX + "PassBandMinX_in", Units.metersToInches(passMinX));
        SmartDashboard.putNumber(DASH_PREFIX + "PassBandMaxX_in", Units.metersToInches(passMaxX));

        SmartDashboard.putString(DASH_PREFIX + "AllianceForZone", isRed ? "Red" : "Blue");
        SmartDashboard.putString(DASH_PREFIX + "AllianceSource", allianceSource);
        SmartDashboard.putBoolean(DASH_PREFIX + "DSAlliancePresent", DriverStation.getAlliance().isPresent());
        SmartDashboard.putNumber(DASH_PREFIX + "ScoreBoundaryX_m", scoreBoundaryX);
        SmartDashboard.putNumber(DASH_PREFIX + "ScoreBoundaryX_in", Units.metersToInches(scoreBoundaryX));
        SmartDashboard.putString(
                DASH_PREFIX + "ScoreRule",
                isRed ? "SCORE side when X >= boundary (and LT held)" : "SCORE side when X <= boundary (and LT held)");

        zoneField.setRobotPose(robotPose);

        // Draw clean zone boundary lines (best readability in Elastic Field widget).
        zoneField.getObject("BlueScoreMaxXLine").setPoses(
                new Pose2d(blueScoreMaxX, 0.0, Rotation2d.kZero),
                new Pose2d(blueScoreMaxX, FieldConstants.FIELD_WIDTH, Rotation2d.kZero));
        zoneField.getObject("RedScoreMinXLine").setPoses(
                new Pose2d(redScoreMinX, 0.0, Rotation2d.kZero),
                new Pose2d(redScoreMinX, FieldConstants.FIELD_WIDTH, Rotation2d.kZero));

        zoneField.getObject("PassBandMinXLine").setPoses(
                new Pose2d(passMinX, 0.0, Rotation2d.kZero),
                new Pose2d(passMinX, FieldConstants.FIELD_WIDTH, Rotation2d.kZero));
        zoneField.getObject("PassBandMaxXLine").setPoses(
                new Pose2d(passMaxX, 0.0, Rotation2d.kZero),
                new Pose2d(passMaxX, FieldConstants.FIELD_WIDTH, Rotation2d.kZero));
        zoneField.getObject("ScoreBoundaryXLine").setPoses(
                new Pose2d(scoreBoundaryX, 0.0, Rotation2d.kZero),
                new Pose2d(scoreBoundaryX, FieldConstants.FIELD_WIDTH, Rotation2d.kZero));

        // Draw full zone outlines so Elastic shows full regions (not just center points).
        zoneField.getObject("BlueScoreZoneOutline").setPoses(
                denseRectOutlinePoses(0.0, blueScoreMaxX, 0.0, FieldConstants.FIELD_WIDTH, 0.4));
        zoneField.getObject("PassZoneOutline").setPoses(
                denseRectOutlinePoses(passMinX, passMaxX, 0.0, FieldConstants.FIELD_WIDTH, 0.4));
        zoneField.getObject("RedScoreZoneOutline").setPoses(
                denseRectOutlinePoses(redScoreMinX, FieldConstants.FIELD_LENGTH, 0.0, FieldConstants.FIELD_WIDTH, 0.4));
    }

    private ZoneBoundaries getZoneBoundaries() {
        // Zone boundary at each hub front-face X.
        // BoundaryOffsetM lets you expand/shrink scoring zone for testing:
        // positive => scoring zones grow toward field center, passing zone narrows.
        // negative => scoring zones shrink, passing zone widens.
        double offset = SmartDashboard.getNumber(DASH_PREFIX + "BoundaryOffsetM", 0.0);
        double blueScoreMaxX = FieldConstants.Hub.BLUE_FRONT_FACE.getX() + offset;
        double redScoreMinX = FieldConstants.Hub.RED_FRONT_FACE.getX() - offset;

        // Keep boundaries valid and non-overlapping.
        blueScoreMaxX = MathUtil.clamp(blueScoreMaxX, 0.0, FieldConstants.FIELD_LENGTH);
        redScoreMinX = MathUtil.clamp(redScoreMinX, 0.0, FieldConstants.FIELD_LENGTH);
        if (blueScoreMaxX > redScoreMinX) {
            double mid = (blueScoreMaxX + redScoreMinX) / 2.0;
            blueScoreMaxX = mid;
            redScoreMinX = mid;
        }

        return new ZoneBoundaries(blueScoreMaxX, redScoreMinX);
    }

    private record ZoneBoundaries(double blueScoreMaxX, double redScoreMinX) {
    }

    private TrajectoryData buildShotTrajectory(Pose3d shooterPose, Pose3d effectiveTarget, InterceptSolution intercept) {
        if (intercept.flightTime() <= 0.0 || intercept.launchSpeed() <= 0.0) {
            return null;
        }

        double dx = effectiveTarget.getX() - shooterPose.getX();
        double dy = effectiveTarget.getY() - shooterPose.getY();
        double yaw = Math.atan2(dy, dx);

        double speed = intercept.launchSpeed();
        double pitch = intercept.launchPitchRad();
        double vHoriz = speed * Math.cos(pitch);
        double vx = vHoriz * Math.cos(yaw);
        double vy = vHoriz * Math.sin(yaw);

        double dt = Math.max(0.01, SmartDashboard.getNumber(DASH_PREFIX + "TrajectoryStepS", 0.05));
        double flightTime = intercept.flightTime();

        List<Pose2d> poses2d = new ArrayList<>();
        List<Pose3d> poses3d = new ArrayList<>();
        double zPeak = shooterPose.getZ();
        for (double t = 0.0; t <= flightTime; t += dt) {
            double x = shooterPose.getX() + vx * t;
            double y = shooterPose.getY() + vy * t;
            double z = shooterPose.getZ() + speed * Math.sin(pitch) * t - 0.5 * 9.81 * t * t;
            zPeak = Math.max(zPeak, z);
            poses2d.add(new Pose2d(x, y, Rotation2d.kZero));
            poses3d.add(new Pose3d(x, y, z, Rotation3d.kZero));
        }

        Pose2d impactPose = poses2d.get(poses2d.size() - 1);

        SmartDashboard.putNumber(DASH_PREFIX + "TrajectoryFlightTimeS", flightTime);
        SmartDashboard.putNumber(DASH_PREFIX + "TrajectoryPeakZ_m", zPeak);
        SmartDashboard.putNumber(DASH_PREFIX + "TrajectoryImpactX_m", impactPose.getX());
        SmartDashboard.putNumber(DASH_PREFIX + "TrajectoryImpactY_m", impactPose.getY());
        return new TrajectoryData(poses2d, poses3d, impactPose);
    }

    private void publishRawTrajectory(TrajectoryData data) {
        if (data == null) {
            zoneField.getObject("ShotTrajectoryRaw").setPoses(List.of());
            zoneField.getObject("ShotImpactRaw").setPoses(List.of());
            trajectory3dRawPublisher.set(new Pose3d[0]);
            return;
        }
        zoneField.getObject("ShotTrajectoryRaw").setPoses(data.poses2d);
        zoneField.getObject("ShotImpactRaw").setPose(data.impactPose);
        trajectory3dRawPublisher.set(data.poses3d.toArray(new Pose3d[0]));
    }

    private void publishAllowedTrajectory(TrajectoryData data, boolean allowed) {
        if (!allowed || data == null) {
            zoneField.getObject("ShotTrajectoryAllowed").setPoses(List.of());
            zoneField.getObject("ShotImpactAllowed").setPoses(List.of());
            trajectory3dAllowedPublisher.set(new Pose3d[0]);
            return;
        }
        zoneField.getObject("ShotTrajectoryAllowed").setPoses(data.poses2d);
        zoneField.getObject("ShotImpactAllowed").setPose(data.impactPose);
        trajectory3dAllowedPublisher.set(data.poses3d.toArray(new Pose3d[0]));
    }

    private void clearShotTrajectory() {
        zoneField.getObject("ShotTrajectoryRaw").setPoses(List.of());
        zoneField.getObject("ShotImpactRaw").setPoses(List.of());
        zoneField.getObject("ShotTrajectoryAllowed").setPoses(List.of());
        zoneField.getObject("ShotImpactAllowed").setPoses(List.of());
        trajectory3dRawPublisher.set(new Pose3d[0]);
        trajectory3dAllowedPublisher.set(new Pose3d[0]);
    }

    private record TrajectoryData(List<Pose2d> poses2d, List<Pose3d> poses3d, Pose2d impactPose) {
    }

    private Pose2d[] denseRectOutlinePoses(double minX, double maxX, double minY, double maxY, double stepM) {
        double step = Math.max(0.05, stepM);
        List<Pose2d> poses = new ArrayList<>();

        for (double x = minX; x <= maxX; x += step) {
            poses.add(new Pose2d(Math.min(x, maxX), minY, Rotation2d.kZero));
        }
        for (double y = minY; y <= maxY; y += step) {
            poses.add(new Pose2d(maxX, Math.min(y, maxY), Rotation2d.kZero));
        }
        for (double x = maxX; x >= minX; x -= step) {
            poses.add(new Pose2d(Math.max(x, minX), maxY, Rotation2d.kZero));
        }
        for (double y = maxY; y >= minY; y -= step) {
            poses.add(new Pose2d(minX, Math.max(y, minY), Rotation2d.kZero));
        }

        poses.add(new Pose2d(minX, minY, Rotation2d.kZero));
        return poses.toArray(new Pose2d[0]);
    }

    private Pose3d getBestPassTarget(Pose2d robotPose, Pose3d hub) {
        // Increase to pass farther to the side of hub; decrease to pass closer to center.
        double offset = SmartDashboard.getNumber(DASH_PREFIX + "PassLateralOffsetM", 1.6);
        double safeEdgeMargin = 0.5;

        double leftY = MathUtil.clamp(hub.getY() + offset, safeEdgeMargin, FieldConstants.FIELD_WIDTH - safeEdgeMargin);
        double rightY = MathUtil.clamp(hub.getY() - offset, safeEdgeMargin, FieldConstants.FIELD_WIDTH - safeEdgeMargin);

        double leftDy = Math.abs(robotPose.getY() - leftY);
        double rightDy = Math.abs(robotPose.getY() - rightY);
        double chosenY = leftDy < rightDy ? leftY : rightY;

        return new Pose3d(hub.getX(), chosenY, hub.getZ(), Rotation3d.kZero);
    }

    private boolean canScoreHubNow() {
        if (SmartDashboard.getBoolean(DASH_PREFIX + "ForceScoringPermission", false)) {
            SmartDashboard.putBoolean(DASH_PREFIX + "ScoringPermissionOverride", true);
            SmartDashboard.putBoolean(DASH_PREFIX + "GoalActiveByShift", true);
            return true;
        }
        SmartDashboard.putBoolean(DASH_PREFIX + "ScoringPermissionOverride", false);

        boolean active = isMyGoalActiveByShift();
        SmartDashboard.putBoolean(DASH_PREFIX + "GoalActiveByShift", active);
        return active;
    }

    private boolean isMyGoalActiveByShift() {
        Alliance alliance = getAllianceForAutoShoot();

        // Hub always active in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        // If not teleop enabled, we do not score.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        SmartDashboard.putString(DASH_PREFIX + "GameDataRaw", gameData);
        SmartDashboard.putNumber(DASH_PREFIX + "MatchTimeS", matchTime);

        boolean redInactiveFirst;
        if ("R".equalsIgnoreCase(gameData)) {
            redInactiveFirst = true;
        } else if ("B".equalsIgnoreCase(gameData)) {
            redInactiveFirst = false;
        } else {
            // No/invalid data yet: assume active to avoid deadlocking scoring.
            return true;
        }

        // Shift 1 is active for blue if red is inactive first, else active for red.
        boolean shift1Active = (alliance == Alliance.Red) ? !redInactiveFirst : redInactiveFirst;

        if (matchTime > 130.0) {
            return true; // Transition shift (active)
        } else if (matchTime > 105.0) {
            return shift1Active; // Shift 1
        } else if (matchTime > 80.0) {
            return !shift1Active; // Shift 2
        } else if (matchTime > 55.0) {
            return shift1Active; // Shift 3
        } else if (matchTime > 30.0) {
            return !shift1Active; // Shift 4
        } else {
            return true; // Endgame shift (active)
        }
    }

    private Alliance getAllianceForAutoShoot() {
        Optional<Alliance> dsAlliance = DriverStation.getAlliance();
        if (dsAlliance.isPresent()) {
            allianceSource = "DriverStation";
            return dsAlliance.get();
        }

        // Sim fallback: DS sim may publish this even when getAlliance() is empty.
        if (FMS_TABLE.containsKey("IsRedAlliance")) {
            boolean isRed = FMS_TABLE.getEntry("IsRedAlliance").getBoolean(false);
            allianceSource = "FMSInfo/IsRedAlliance";
            return isRed ? Alliance.Red : Alliance.Blue;
        }

        allianceSource = "DefaultBlue";
        return Alliance.Blue;
    }

    private void stopFeedAndShooter() {
        spindex.stop();
        ballElevator.stop();
        shooter.stop();
    }

    private void putDashboardDefaults() {
        // Scoring/pass zone boundary tuning around hub front-face X.
        SmartDashboard.putNumber(DASH_PREFIX + "BoundaryOffsetM", 0.0);
        SmartDashboard.putNumber(DASH_PREFIX + "PassLateralOffsetM", 1.6);

        // Readiness tuning: tighter = better accuracy, looser = higher throughput.
        SmartDashboard.putNumber(DASH_PREFIX + "ScoreSpeedToleranceRps", 1.0);
        SmartDashboard.putNumber(DASH_PREFIX + "PassSpeedToleranceRps", 2.5);
        SmartDashboard.putNumber(DASH_PREFIX + "ScoreAimToleranceDeg", 2.5);
        SmartDashboard.putNumber(DASH_PREFIX + "PassAimToleranceDeg", 6.0);

        // Safety/performance limits.
        // MaxDistance: prevents launching from very poor locations.
        // FeedPercent: higher is faster ball flow; too high can reduce consistency.
        SmartDashboard.putNumber(DASH_PREFIX + "ScoreMaxDistanceM", 6.0);
        SmartDashboard.putNumber(DASH_PREFIX + "PassMaxDistanceM", 10.0);
        SmartDashboard.putNumber(DASH_PREFIX + "ScoreFeedPercent", 0.85);
        SmartDashboard.putNumber(DASH_PREFIX + "PassFeedPercent", 1.0);

        // Pass launch speed override used only in PASS mode.
        // Raise if passes are short; lower if passes overshoot.
        SmartDashboard.putNumber(DASH_PREFIX + "PassShotSpeedRps", 11.0);
        SmartDashboard.putNumber(DASH_PREFIX + "TrajectoryStepS", 0.05);
        SmartDashboard.putString(DASH_PREFIX + "AllianceSource", "Unknown");
        SmartDashboard.putBoolean(DASH_PREFIX + "DSAlliancePresent", false);
        SmartDashboard.putBoolean(DASH_PREFIX + "ForceScoringPermission", false);
        SmartDashboard.putBoolean(DASH_PREFIX + "ScoringPermissionOverride", false);

    }
}
