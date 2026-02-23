package frc.robot;

import static edu.wpi.first.units.Units.Grams;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import com.techhounds.houndutil.houndauto.Reflector;
import com.techhounds.houndutil.houndlib.BallConstants;
import com.techhounds.houndutil.houndlib.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(651.22);
    public static final double FIELD_WIDTH = Units.inchesToMeters(317.69);

    public static Pose2d rotateBluePoseIfNecessary(Pose2d original) {
        return Utils.shouldFlipValueToRed()
                ? Reflector.rotatePoseAcrossField(original, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH)
                : original;
    }

    public static final BallConstants BALL_CONSTANTS = new BallConstants(
            Grams.of(210).in(Kilograms), Inches.of(3).in(Meters), 1.2, 0.30, 1.2, 0.35, 9.81, 20);

    public static final class Hub {
        // Blue hub target used by SCORE mode.
        // Editing this pose moves where all blue-alliance hub shots aim:
        // X/Y shift left-right/forward-back on field, Z changes required hood/shot arc.
        // Keep in meters; these are converted from official inch measurements.
        public static final Pose3d BLUE_CENTER = new Pose3d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72), Rotation3d.kZero);

        public static final Pose3d BLUE_FRONT_FACE = new Pose3d(Units.inchesToMeters(158.50), Units.inchesToMeters(158.84),
                Units.inchesToMeters(0), Rotation3d.kZero);

        // Red hub is the blue hub reflected across the field center (180 deg rotation).
        public static final Pose3d RED_CENTER = new Pose3d(
                FIELD_LENGTH - BLUE_CENTER.getX(),
                FIELD_WIDTH - BLUE_CENTER.getY(),
                BLUE_CENTER.getZ(),
                Rotation3d.kZero);

        // Red hub is the blue hub reflected across the field center (180 deg rotation).
        public static final Pose3d RED_FRONT_FACE = new Pose3d(
                FIELD_LENGTH - BLUE_FRONT_FACE.getX(),
                FIELD_WIDTH - BLUE_FRONT_FACE.getY(),
                BLUE_FRONT_FACE.getZ(),
                Rotation3d.kZero);

        // Backward-compatibility alias for existing callsites.
        public static final Pose3d CENTER = BLUE_CENTER;

        public static Pose3d getCenterForAlliance(Optional<Alliance> alliance) {
            return alliance.orElse(Alliance.Blue) == Alliance.Red ? RED_CENTER : BLUE_CENTER;
        }
    }
}
