// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSystems.Hood;
import frc.robot.subsystems.ShooterSystems.Shooter;
import frc.robot.subsystems.ShooterSystems.ShotCalculator;
import frc.robot.subsystems.ShooterSystems.Turret;
public class RobotContainer 
{
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed, change multiplier to change max speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);      // Drive controller
    private final CommandXboxController operator = new CommandXboxController(1);    // Operator controller
    private final CommandXboxController test = new CommandXboxController(2);        // Test controller
    private final CommandXboxController test2 = new CommandXboxController(3);       // Second test controller for testing subsystems with more complex controls (e.g. shooter)

    private static final double TEST_SPINDEX_STEP = 0.1;
    private static final double TEST_SHOOTER_STEP = 0.1;
    private static final double TEST_SHOOTER_MAX_SPEED_RPS = 40.0;
    private double testSpindexPercent = 0.0;
    private double testShooterPercent = 0.0;

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

    /*
    private final AutoShoot autoShootCommand = new AutoShoot(
            drivetrain,
            s_shooter,
            s_hood,
            s_turret,
            s_spindex,
            s_ballElevator,
            s_shotCalculator,
            () -> !RobotState.isTest() && operator.getLeftTriggerAxis() <= 0.5, // Inverted LT behavior: AutoShoot runs unless LT is held.
            () -> 
            {
                if (RobotState.isTest()) {
                    return false;
                }
                // Sim helper: allow forcing score-enable without a physical LT input.
                if (RobotBase.isSimulation()
                        && SmartDashboard.getBoolean("AutoShoot/ForceScoreEnable", false)) {
                    return true;
                }
                // Match inverted LT behavior: score-enable while LT is NOT held.
                return operator.getLeftTriggerAxis() <= 0.5;
            }
    );
    */
    
    private final SendableChooser<Command> autoChooser; 

    public RobotContainer() 
    {
        SmartDashboard.putBoolean("AutoShoot/ForceScoreEnable", false);
        SmartDashboard.putNumber("Test/SpindexPercent", testSpindexPercent);
        SmartDashboard.putNumber("Test/ShooterPercent", testShooterPercent);
        SmartDashboard.putNumber("Test/ShooterSetpointRPS", 0.0);
        configureBindings();

        if (drivetrain.isAutoBuilderConfigured()) {
            autoChooser = AutoBuilder.buildAutoChooser();
        } else {
            autoChooser = new SendableChooser<>();
            autoChooser.setDefaultOption("No Auto (PathPlanner config missing)", null);
        }
        
        SmartDashboard.putData("Auto Mode", autoChooser); 
    }

    private void configureBindings() 
    {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand
        (
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest
            (() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //s_shooter.setDefaultCommand(autoShootCommand.onlyIf(() -> !RobotState.isTest()));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        

        driver.a().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        driver.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); // Reset field centric heading to 0

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //driver.leftBumper().whileTrue(); // Intake Left
        //driver.rightBumper().whileTrue(); // Intake Right

        //driver.rightTrigger().whileTrue(); // Manual Shoot

        drivetrain.registerTelemetry(logger::telemeterize);

        //TEST:
        //CLimber:
        test.a().onTrue(s_climber.runOnce(() -> s_climber.setHeightInches(Constants.Climber.MIN_HEIGHT_INCHES))); // a goes to minimum climber height
        test.y().onTrue(s_climber.runOnce(() -> s_climber.setHeightInches(Constants.Climber.MAX_HEIGHT_INCHES))); // y goes to max climber height
        // Right Intake
        test.b().onTrue(r_Intake.runOnce(() -> r_Intake.setAngle(Constants.RightIntake.MAX_PIVOT_ANGLE)));
        test.rightTrigger().whileTrue(r_Intake.runOnce(() -> r_Intake.runFeed(0.5)));
        test.rightTrigger().whileFalse(r_Intake.runOnce(() -> r_Intake.stop())); 
         // testing at 50% speed
        // Left Intake
        test.x().onTrue(r_Intake.runOnce(() -> l_Intake.setAngle(Constants.LeftIntake.MAX_PIVOT_ANGLE)));
        test.leftTrigger().whileTrue(l_Intake.runOnce(() -> l_Intake.runFeed(0.5))); // testing at 50% speed
        test.leftTrigger().whileFalse(l_Intake.runOnce(() -> l_Intake.stop()));
        //Spindex:
        test.povUp().onTrue(s_spindex.runOnce(() -> {
            s_spindex.setSpeed(Constants.Spindexer.MAX_SPINDEX_SPEED_RPS*75);
        }));
        test.povDown().onTrue(s_spindex.runOnce(() -> {
            s_spindex.setDumbSpeed(0);
        }));
        //Shooter
        test.rightBumper().onTrue(s_shooter.runOnce(() -> {
            s_shooter.setDumbSpeed(1); // testing at 50% speed
        }));
        test.leftBumper().onTrue(s_shooter.runOnce(() -> {
            s_shooter.setDumbSpeed(0.0); // stop shooter
        }));
        // Test Ball Elevator
        test.povRight().onTrue(s_ballElevator.runOnce(() -> {
            s_ballElevator.setSpeed(129);
        }));
        test.povLeft().onTrue(s_ballElevator.runOnce(() -> {
            s_ballElevator.setSpeed(0);
        }));

        // TEST Hood
        // TEST TURRET

        // TEST 2 Controls:
        test2.a().onTrue(s_hood.runOnce(() -> s_hood.zeroEncoder()).ignoringDisable(true));
        test2.b().onTrue(s_turret.runOnce(() -> s_turret.zeroEncoder()).ignoringDisable(true));
        test2.povUp().onTrue(l_Intake.runOnce(() -> l_Intake.zeroEncoder()).ignoringDisable(true));
        test2.povDown().onTrue(r_Intake.runOnce(() -> r_Intake.zeroEncoder()).ignoringDisable(true));
        test2.y().onTrue(s_climber.runOnce(() -> s_climber.zeroEncoder()).ignoringDisable(true));
        test2.x().onTrue(s_turret.runOnce(() -> s_turret.zeroEncoder()).ignoringDisable(true));
    }

    public Command getAutonomousCommand() 
    {
        return autoChooser.getSelected(); 
    }

    public void printDiagnostics() 
    {
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
}
