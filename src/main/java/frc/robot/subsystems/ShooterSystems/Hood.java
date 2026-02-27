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

public class Hood extends SubsystemBase
{
    private static final double DEGREES_PER_ROTATION = 360.0;
    private static final int CONFIG_RETRIES = 5;
    
    private static final double STATOR_CURRENT_LIMIT_AMPS = 10.0;   // TODO - Adjust as necessary to prevent damage to the motor and mechanism - set low rn for testing purposes
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 10.0;   // TODO - Adjust as necessary to prevent damage to the motor and mechanism - set low rn for testing purposes
    private static final double MM_CRUISE_RPS = 5.0;
    private static final double MM_ACCEL_RPS2 = 5.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 78.33; // TOOD - Check this : Motor Gearbox = 20:1, 24T Gear -> 48T Gear = 2:1, 24T Pulley -> 47T Pulley = 47/24:1, Total Ratio = 20 * 2 * 47/24 = 78.33:1
    private static final double SLOT0_KS = 0.0;
    private static final double SLOT0_KV = 0.0;
    private static final double SLOT0_KP = 10.0;
    private static final double SLOT0_KI = 0.0;
    private static final double SLOT0_KD = 1.0;
    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;

    private double currentAngleSetPoint = 0.0; // Store the current angle preset for alignment checks

    private final TalonFX hood_rotation_motor;
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public Hood()
    {
        hood_rotation_motor = new TalonFX(Constants.CAN_ID.HOOD_ROTATE, Constants.CAN_BUS.CANIVORE);
        configureHoodMotor();
    }

    public void zeroEncoder() 
    {
        hood_rotation_motor.setPosition(0.0);
    }

    public void setAngle(double angleDegrees) 
    {
        double targetRotations = angleDegrees / DEGREES_PER_ROTATION;
        hood_rotation_motor.setControl(motionMagicVoltage.withPosition(targetRotations));
        currentAngleSetPoint = angleDegrees; // Degrees, for alignment checks and dashboard readability.
        SmartDashboard.putNumber("Hood Set Point", angleDegrees);
    }

    public double getAngle()
    {
        return hood_rotation_motor.getPosition().getValueAsDouble() * DEGREES_PER_ROTATION;
    }

    public double getAngleSetpoint()
    {
        return currentAngleSetPoint;
    }

    public boolean isAligned()
    {
        return isAligned(Constants.Hood.AngleToleranceDegrees);
    }

    public boolean isAligned(double toleranceDegrees)
    {
        return Math.abs(getAngle() - currentAngleSetPoint) <= toleranceDegrees;
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void configureHoodMotor() 
    {
        TalonFXConfiguration hoodConfigs = new TalonFXConfiguration()
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

        hoodConfigs.Slot0.kS = SLOT0_KS;
        hoodConfigs.Slot0.kV = SLOT0_KV;
        hoodConfigs.Slot0.kP = SLOT0_KP;
        hoodConfigs.Slot0.kI = SLOT0_KI;
        hoodConfigs.Slot0.kD = SLOT0_KD;
        hoodConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = hood_rotation_motor.getConfigurator().apply(hoodConfigs);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.out.println("Could not apply hood configs, error code: " + status);
        }
    }
}
