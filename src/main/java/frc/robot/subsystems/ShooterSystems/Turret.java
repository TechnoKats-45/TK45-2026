package frc.robot.subsystems.ShooterSystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase
{
    private static final double DEGREES_PER_ROTATION = 360.0;
    private static final int CONFIG_RETRIES = 5;
    private static final double STATOR_CURRENT_LIMIT_AMPS = 10.0;   // Adjust as necessary to prevent damage to the motor and mechanism - set low rn for testing purposes
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 10.0;   // Adjust as necessary to prevent damage to the motor and mechanism - set low rn for testing purposes
    private static final double MM_CRUISE_RPS = 5.0;
    private static final double MM_ACCEL_RPS2 = 5.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 91.36; // TODO - Check this: 16:1 Sport Gearbox + 35T Gear -> 200T Gear = 5.71:1, Total Reduction = 91.36:1
    private static final double SLOT0_KS = 0.0;
    private static final double SLOT0_KV = 0.0;
    private static final double SLOT0_KP = 10.0;
    private static final double SLOT0_KI = 0.0;
    private static final double SLOT0_KD = 1.0;
    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;

    private double currentAngleSetPoint = 0.0; // Store the current angle preset for alignment checks

    private final TalonFX turret_rotation_motor;
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public Turret()
    {
        turret_rotation_motor = new TalonFX(Constants.CAN_ID.TURRET, Constants.CAN_BUS.RIO);
        configureTurretMotor();
    }

    public void zeroEncoder() 
    {
        turret_rotation_motor.setPosition(0.0);
    }

    public void setAngle(double angleDegrees) 
    {
        double targetRotations = angleDegrees / DEGREES_PER_ROTATION;
        turret_rotation_motor.setControl(motionMagicVoltage.withPosition(targetRotations));
        currentAngleSetPoint = angleDegrees; // Degrees, for alignment checks and dashboard readability.
        SmartDashboard.putNumber("Turret Set Point", angleDegrees);
    }

    public double getAngle()
    {
        return turret_rotation_motor.getPosition().getValueAsDouble() * DEGREES_PER_ROTATION;
    }

    public double getAngleSetpoint()
    {
        return currentAngleSetPoint;
    }

    public boolean isAligned()
    {
        return isAligned(Constants.Turret.AngleToleranceDegrees);
    }

    public boolean isAligned(double toleranceDegrees)
    {
        return Math.abs(getAngle() - currentAngleSetPoint) <= toleranceDegrees;
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void configureTurretMotor() 
    {
        TalonFXConfiguration turretConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS)
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
                        .withStatorCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(MM_CRUISE_RPS))
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MM_ACCEL_RPS2)));

        turretConfigs.Slot0.kS = SLOT0_KS;
        turretConfigs.Slot0.kV = SLOT0_KV;
        turretConfigs.Slot0.kP = SLOT0_KP;
        turretConfigs.Slot0.kI = SLOT0_KI;
        turretConfigs.Slot0.kD = SLOT0_KD;
        turretConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turretConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = turret_rotation_motor.getConfigurator().apply(turretConfigs);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.out.println("Could not apply turret configs, error code: " + status);
        }
    }
}
