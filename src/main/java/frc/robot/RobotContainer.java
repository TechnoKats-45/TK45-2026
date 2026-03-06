// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Photon Vision IP: http://10.0.45.11:5800/
//               or:  http://photonvision.local:5800/#/dashboard
package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoFeed;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSystems.BallElevator;
import frc.robot.subsystems.ShooterSystems.Hood;
import frc.robot.subsystems.ShooterSystems.Shooter;
import frc.robot.subsystems.ShooterSystems.ShotCalculator;
import frc.robot.subsystems.ShooterSystems.Turret;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;


public class RobotContainer 
{
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed, change multiplier to change max speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);      // Drive controller
    private final CommandXboxController operator = new CommandXboxController(1);    // Operator controller
    private final CommandXboxController test = new CommandXboxController(2);        // Test controller
    private final CommandXboxController test2 = new CommandXboxController(3);       // Second test controller for testing subsystems with more complex controls (e.g. shooter)

    private static final double TEST_SPINDEX_STEP = 0.1;
    private static final double TEST_SHOOTER_STEP = 0.1;
    private static final double TEST_SHOOTER_MAX_SPEED_RPS = 40.0;
    private static final double INTAKE_UP_TOLERANCE_ROTATIONS = 0.02;
    private static final double INTAKE_SPIN_ENABLE_TOLERANCE_ROTATIONS = 0.08;
    private double testSpindexPercent = 0.0;
    private double testShooterPercent = 0.0;
    private boolean rightIntakePivotDown = false;
    private boolean leftIntakePivotDown = false;
    private boolean prevOperatorA = false;
    private boolean prevOperatorY = false;

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Climber s_climber = new Climber();
    public final Shooter s_shooter = new Shooter();
    public final Hood s_hood = new Hood();
    public final Turret s_turret = new Turret();
    public final Spindexer s_spindex = new Spindexer();
    public final BallElevator s_ballElevator = new BallElevator();
    public final ShotCalculator s_shotCalculator = new ShotCalculator(drivetrain);
    public final RightIntake r_Intake = new RightIntake();
    public final LeftIntake l_Intake = new LeftIntake();
    public final Vision s_vision = new Vision(drivetrain);
    private final AutoAim autoAimCommand = new AutoAim(
            drivetrain,
            s_shooter,
            s_hood,
            s_turret,
            s_shotCalculator);
    private final AutoFeed autoFeedCommand = new AutoFeed(s_spindex, s_ballElevator);
    
    private final SendableChooser<Command> autoChooser; 

    public RobotContainer() 
    {
        registerNamedCommands();

        SmartDashboard.putBoolean("AutoAim/Enabled", true);
        SmartDashboard.putBoolean("AutoAim/RunCommand", false);
        SmartDashboard.putBoolean("AutoAim/UsePassingTarget", false);
        SmartDashboard.putBoolean("AutoIntake/Enabled", true);
        SmartDashboard.putBoolean("AutoIntake/AutoStowEnabled", true);
        SmartDashboard.putBoolean("Drive/UseTestController", false);
        SmartDashboard.putBoolean("ManualShooter/Enabled", false);
        SmartDashboard.putNumber("ManualShooter/SpeedRps", 0.0);
        SmartDashboard.putNumber("ManualShooter/HoodDeg", 0.0);
        SmartDashboard.putNumber("Test/SpindexPercent", testSpindexPercent);
        SmartDashboard.putNumber("Test/ShooterPercent", testShooterPercent);
        SmartDashboard.putNumber("Test/ShooterSetpointRPS", 0.0);

        l_Intake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED);
        r_Intake.setAngle(Constants.RightIntake.PIVOT_ANGLE_UP_STOWED);
        l_Intake.stop();
        r_Intake.stop();

        configureBindings();
        configureDashboardZeroToggles();

        if (drivetrain.isAutoBuilderConfigured()) {
            autoChooser = AutoBuilder.buildAutoChooser();
        } else {
            autoChooser = new SendableChooser<>();
            autoChooser.setDefaultOption("No Auto (PathPlanner config missing)", null);
        }
        
        SmartDashboard.putData("Auto Mode", autoChooser); 
    }

    private void configureDashboardZeroToggles() {
        SmartDashboard.putBoolean("Zero/Hood Encoder", false);
        SmartDashboard.putBoolean("Zero/Turret Encoder", false);
        SmartDashboard.putBoolean("Zero/Left Intake Pivot Encoder", false);
        SmartDashboard.putBoolean("Zero/Right Intake Pivot Encoder", false);
        SmartDashboard.putBoolean("Zero/Climber Encoder", false);
    }

    public void processDashboardZeroRequests() {
        if (SmartDashboard.getBoolean("Zero/Hood Encoder", false)) {
            s_hood.zeroEncoder();
            SmartDashboard.putBoolean("Zero/Hood Encoder", false);
        }
        if (SmartDashboard.getBoolean("Zero/Turret Encoder", false)) {
            s_turret.zeroEncoder();
            SmartDashboard.putBoolean("Zero/Turret Encoder", false);
        }
        if (SmartDashboard.getBoolean("Zero/Left Intake Pivot Encoder", false)) {
            l_Intake.zeroEncoder();
            SmartDashboard.putBoolean("Zero/Left Intake Pivot Encoder", false);
        }
        if (SmartDashboard.getBoolean("Zero/Right Intake Pivot Encoder", false)) {
            r_Intake.zeroEncoder();
            SmartDashboard.putBoolean("Zero/Right Intake Pivot Encoder", false);
        }
        if (SmartDashboard.getBoolean("Zero/Climber Encoder", false)) {
            s_climber.zeroEncoder();
            SmartDashboard.putBoolean("Zero/Climber Encoder", false);
        }
    }

    public void processDashboardManualShooterRequests() {
        if (!SmartDashboard.getBoolean("ManualShooter/Enabled", false)) {
            return;
        }
        if (!DriverStation.isEnabled()) {
            return;
        }
        double speedRps = SmartDashboard.getNumber("ManualShooter/SpeedRps", 0.0);
        double hoodDeg = SmartDashboard.getNumber("ManualShooter/HoodDeg", 0.0);
        s_shooter.setSpeed(speedRps);
        s_hood.setAngle(hoodDeg);
    }

    public void stowIntakesAndStop() {
        l_Intake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED);
        r_Intake.setAngle(Constants.RightIntake.PIVOT_ANGLE_UP_STOWED);
        l_Intake.stop();
        r_Intake.stop();
    }

    public void registerNamedCommands()
    {
        NamedCommands.registerCommand
        (
            "Fire8Balls",   // TODO FIX THIS
            new SequentialCommandGroup(
                new InstantCommand(() -> s_shooter.setSpeed(Constants.Shooter.MAX_SPEED_RPS)),
                new WaitCommand(1), // get shooter up to speed
                new InstantCommand(()->s_ballElevator.runFeed(INTAKE_SPIN_ENABLE_TOLERANCE_ROTATIONS), s_ballElevator),
                new InstantCommand(() -> s_spindex.setSpeed(Constants.Spindexer.MAX_SPINDEX_SPEED_RPS * 0.4)),
                new InstantCommand(() -> s_shooter.stop())
            )
        );
    }

    private void configureBindings() 
    {
        drivetrain.registerTelemetry(logger::telemeterize);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand
        (
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest
            (() ->
                drive.withVelocityX(-getDriveController().getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-getDriveController().getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-getDriveController().getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.a().onTrue(s_vision.runOnce(() -> s_vision.resetPoseFromVision()));
        driver.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); // Reset field centric heading to 0

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        Command leftDriverIntake = Commands.run(() -> {
            if (isAutoIntakeAutoStowEnabled()) {
                r_Intake.setAngle(Constants.RightIntake.PIVOT_ANGLE_UP_STOWED);
                r_Intake.stop();
                if (isRightIntakeUp()) {
                    l_Intake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_DOWN);
                    if (isLeftIntakeDownEnoughToSpin()) {
                        l_Intake.runFeed(Constants.LeftIntake.INTAKE_SPEED);
                    } else {
                        l_Intake.stop();
                    }
                } else {
                    l_Intake.stop();
                }
            } else {
                l_Intake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_DOWN);
                if (isLeftIntakeDownEnoughToSpin()) {
                    l_Intake.runFeed(Constants.LeftIntake.INTAKE_SPEED);
                } else {
                    l_Intake.stop();
                }
            }
        }, l_Intake, r_Intake);

        Command rightDriverIntake = Commands.run(() -> {
            if (isAutoIntakeAutoStowEnabled()) {
                l_Intake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED);
                l_Intake.stop();
                if (isLeftIntakeUp()) {
                    r_Intake.setAngle(Constants.RightIntake.PIVOT_ANGLE_DOWN);
                    if (isRightIntakeDownEnoughToSpin()) {
                        r_Intake.runFeed(Constants.RightIntake.INTAKE_SPEED);
                    } else {
                        r_Intake.stop();
                    }
                } else {
                    r_Intake.stop();
                }
            } else {
                r_Intake.setAngle(Constants.RightIntake.PIVOT_ANGLE_DOWN);
                if (isRightIntakeDownEnoughToSpin()) {
                    r_Intake.runFeed(Constants.RightIntake.INTAKE_SPEED);
                } else {
                    r_Intake.stop();
                }
            }
        }, l_Intake, r_Intake);

        driver.leftBumper().whileTrue(leftDriverIntake.onlyIf(this::isAutoIntakeEnabled));
        driver.leftBumper().onFalse(l_Intake.runOnce(() -> {
            l_Intake.stop();
            if (isAutoIntakeEnabled() && isAutoIntakeAutoStowEnabled()) {
                l_Intake.setAngle(Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED);
            }
        }));

        driver.rightBumper().whileTrue(rightDriverIntake.onlyIf(this::isAutoIntakeEnabled));
        driver.rightBumper().onFalse(r_Intake.runOnce(() -> {
            r_Intake.stop();
            if (isAutoIntakeEnabled() && isAutoIntakeAutoStowEnabled()) {
                r_Intake.setAngle(Constants.RightIntake.PIVOT_ANGLE_UP_STOWED);
            }
        }));
        driver.povUp().whileTrue(s_ballElevator.runOnce(() -> s_ballElevator.dumbSpeed(1)));
        driver.povUp().onFalse(s_ballElevator.runOnce(() -> s_ballElevator.stop()));

        driver.leftTrigger().whileTrue(autoAimCommand);
        driver.rightTrigger().whileTrue(autoFeedCommand);
        operator.povUp().whileTrue(s_climber.run(() -> s_climber.setManualPercent(1)));
        operator.povUp().onFalse(s_climber.runOnce(s_climber::stop));
        operator.povDown().whileTrue(s_climber.run(() -> s_climber.setManualPercent(-1)));  // TODO
        operator.povDown().onFalse(s_climber.runOnce(s_climber::stop)); // TODO
        /*
        //TEST:
        //CLimber:
        test.a().onTrue(s_climber.runOnce(() -> s_climber.setHeightInches(Constants.Climber.MIN_HEIGHT_INCHES))); // a goes to minimum climber height
        test.y().onTrue(s_climber.runOnce(() -> s_climber.setHeightInches(Constants.Climber.MAX_HEIGHT_INCHES))); // y goes to max climber height

        // Right Intake
        test.b().onTrue(r_Intake.runOnce(() -> {
            rightIntakePivotDown = !rightIntakePivotDown;
            r_Intake.setAngle(rightIntakePivotDown
                    ? Constants.RightIntake.PIVOT_ANGLE_DOWN
                    : Constants.RightIntake.PIVOT_ANGLE_UP_STOWED);
        }));
        test.rightBumper().whileTrue(r_Intake.runOnce(() -> r_Intake.runFeed(-1.0)));  // testing at 100% speed
        test.rightBumper().whileFalse(r_Intake.runOnce(() -> r_Intake.stop())); 
        
        // Left Intake
        test.x().onTrue(l_Intake.runOnce(() -> {
            leftIntakePivotDown = !leftIntakePivotDown;
            l_Intake.setAngle(leftIntakePivotDown
                    ? Constants.LeftIntake.PIVOT_ANGLE_DOWN
                    : Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED);
        }));
        test.leftBumper().whileTrue(l_Intake.runOnce(() -> l_Intake.runFeed(1)));
        test.leftBumper().whileFalse(l_Intake.runOnce(() -> l_Intake.stop()));

        //Spindex:
        test.povUp().onTrue(s_spindex.runOnce(() -> {
            s_spindex.setSpeed(Constants.Spindexer.MAX_SPINDEX_SPEED_RPS*.4);   // was /4
        }));
        test.povDown().onTrue(s_spindex.runOnce(() -> {
            s_spindex.stop();
        }));
        //Shooter
        test.rightTrigger().onTrue(s_shooter.runOnce(() -> {
            s_shooter.setSpeed(Constants.Shooter.MAX_SPEED_RPS);
        }));
        test.rightTrigger().onFalse(s_shooter.runOnce(() -> {
            s_shooter.stop(); // stop shooter
        }));
        // Test Ball Elevator
        test.povRight().onTrue(s_ballElevator.runOnce(() -> {
            s_ballElevator.runFeed(1);
        }));
        test.povLeft().onTrue(s_ballElevator.runOnce(() -> {
            s_ballElevator.stop();
        }));
        */
    }

    public Command getAutonomousCommand() 
    {
        return autoChooser.getSelected(); 
    }

    public void printDiagnostics() 
    {
        SmartDashboard.putString("AutoAim/TargetMode", autoAimCommand.getLastTargetMode());
        SmartDashboard.putBoolean("AutoAim/AutoEnabled", isAutoAimEnabled());
        var allianceHubCenter = FieldConstants.Hub.getCenterForAlliance(DriverStation.getAlliance());
        var robotPose = drivetrain.getState().Pose;
        double rangeToHubM = robotPose.getTranslation().getDistance(allianceHubCenter.toPose2d().getTranslation());
        SmartDashboard.putNumber("RangeToHubM", rangeToHubM);
        SmartDashboard.putNumber("RangeToHubIn", edu.wpi.first.math.util.Units.metersToInches(rangeToHubM));

        s_spindex.printDiagnostics();
        s_ballElevator.printDiagnostics();
        
        s_shooter.printDiagnostics();
        s_turret.printDiagnostics();
        s_hood.printDiagnostics();
        //s_shotCalculator.printDiagnostics();

        r_Intake.printDiagnostics();
        l_Intake.printDiagnostics();
        s_climber.printDiagnostics();

        // Add more subsystem diagnostics as needed //TODO
    }

    private boolean isLeftIntakeUp() {
        return Math.abs(l_Intake.getAngle() - Constants.LeftIntake.PIVOT_ANGLE_UP_STOWED)
                <= INTAKE_UP_TOLERANCE_ROTATIONS;
    }

    private boolean isRightIntakeUp() {
        return Math.abs(r_Intake.getAngle() - Constants.RightIntake.PIVOT_ANGLE_UP_STOWED)
                <= INTAKE_UP_TOLERANCE_ROTATIONS;
    }

    private boolean isLeftIntakeDownEnoughToSpin() {
        return Math.abs(l_Intake.getAngle() - Constants.LeftIntake.PIVOT_ANGLE_DOWN)
                <= INTAKE_SPIN_ENABLE_TOLERANCE_ROTATIONS;
    }

    private boolean isRightIntakeDownEnoughToSpin() {
        return Math.abs(r_Intake.getAngle() - Constants.RightIntake.PIVOT_ANGLE_DOWN)
                <= INTAKE_SPIN_ENABLE_TOLERANCE_ROTATIONS;
    }

    private boolean isAutoIntakeAutoStowEnabled() {
        return SmartDashboard.getBoolean("AutoIntake/AutoStowEnabled", true);
    }

    private boolean isAutoIntakeEnabled() {
        return SmartDashboard.getBoolean("AutoIntake/Enabled", true);
    }

    private boolean isAutoAimEnabled() {
        return SmartDashboard.getBoolean("AutoAim/Enabled", true);
    }

    private CommandXboxController getDriveController() {
        return SmartDashboard.getBoolean("Drive/UseTestController", false) ? test : driver;
    }

    /**
     * Returns +1 on operator A press edge, -1 on operator Y press edge, else 0.
     */
    public int consumeClimberServoNudgeDirection() {
        boolean aNow = operator.getHID().getAButton();
        boolean yNow = operator.getHID().getYButton();
        boolean aPressed = aNow && !prevOperatorA;
        boolean yPressed = yNow && !prevOperatorY;
        prevOperatorA = aNow;
        prevOperatorY = yNow;

        if (aPressed && !yPressed) {
            return 1;
        }
        if (yPressed && !aPressed) {
            return -1;
        }
        return 0;
    }
}
