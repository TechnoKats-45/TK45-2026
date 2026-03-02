package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LeftIntake;
import frc.robot.subsystems.RightIntake;

public class AutoIntake extends Command {
    private enum IntakeSide {
        LEFT,
        RIGHT,
        NONE
    }

    private static final double LATERAL_SPEED_SWITCH_THRESHOLD_MPS = 0.15;
    private static final double INTAKE_UP_TOLERANCE_ROTATIONS = 0.02;

    private final Drivetrain drivetrain;
    private final LeftIntake leftIntake;
    private final RightIntake rightIntake;

    private IntakeSide activeSide = IntakeSide.NONE;

    public AutoIntake(Drivetrain drivetrain, LeftIntake leftIntake, RightIntake rightIntake) {
        this.drivetrain = drivetrain;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        addRequirements(leftIntake, rightIntake);
    }

    @Override
    public void initialize() {
        activeSide = IntakeSide.NONE;
        commandBothUpAndStop();
    }

    @Override
    public void execute() {
        IntakeSide desiredSide = getDesiredSideFromTravel();

        if (desiredSide == IntakeSide.NONE) {
            desiredSide = activeSide;
        }

        if (desiredSide != activeSide) {
            if (activeSide == IntakeSide.LEFT) {
                leftIntake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED);
                leftIntake.stop();
                if (isLeftUp()) {
                    activeSide = IntakeSide.NONE;
                } else {
                    rightIntake.stop();
                    return;
                }
            } else if (activeSide == IntakeSide.RIGHT) {
                rightIntake.setAngle(Constants.RightIntake.PIVOT_ANGLE_UP_STOWED);
                rightIntake.stop();
                if (isRightUp()) {
                    activeSide = IntakeSide.NONE;
                } else {
                    leftIntake.stop();
                    return;
                }
            }
        }

        if (activeSide == IntakeSide.NONE) {
            if (desiredSide == IntakeSide.LEFT) {
                rightIntake.setAngle(Constants.RightIntake.PIVOT_ANGLE_UP_STOWED);
                rightIntake.stop();
                if (isRightUp()) {
                    activeSide = IntakeSide.LEFT;
                } else {
                    leftIntake.stop();
                    return;
                }
            } else if (desiredSide == IntakeSide.RIGHT) {
                leftIntake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED);
                leftIntake.stop();
                if (isLeftUp()) {
                    activeSide = IntakeSide.RIGHT;
                } else {
                    rightIntake.stop();
                    return;
                }
            }
        }

        if (activeSide == IntakeSide.LEFT) {
            leftIntake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_DOWN);
            leftIntake.runFeed(Constants.LeftIntake.INTAKE_SPEED);
            rightIntake.setAngle(Constants.RightIntake.PIVOT_ANGLE_UP_STOWED);
            rightIntake.stop();
        } else if (activeSide == IntakeSide.RIGHT) {
            rightIntake.setAngle(Constants.RightIntake.PIVOT_ANGLE_DOWN);
            rightIntake.runFeed(Constants.RightIntake.INTAKE_SPEED);
            leftIntake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED);
            leftIntake.stop();
        } else {
            commandBothUpAndStop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        commandBothUpAndStop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private IntakeSide getDesiredSideFromTravel() {
        double vy = drivetrain.getState().Speeds.vyMetersPerSecond;
        if (Math.abs(vy) < LATERAL_SPEED_SWITCH_THRESHOLD_MPS) {
            return IntakeSide.NONE;
        }
        return vy > 0.0 ? IntakeSide.LEFT : IntakeSide.RIGHT;
    }

    private boolean isLeftUp() {
        return Math.abs(leftIntake.getAngle() - Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED)
                <= INTAKE_UP_TOLERANCE_ROTATIONS;
    }

    private boolean isRightUp() {
        return Math.abs(rightIntake.getAngle() - Constants.RightIntake.PIVOT_ANGLE_UP_STOWED)
                <= INTAKE_UP_TOLERANCE_ROTATIONS;
    }

    private void commandBothUpAndStop() {
        leftIntake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED);
        rightIntake.setAngle(Constants.RightIntake.PIVOT_ANGLE_UP_STOWED);
        leftIntake.stop();
        rightIntake.stop();
    }
}
