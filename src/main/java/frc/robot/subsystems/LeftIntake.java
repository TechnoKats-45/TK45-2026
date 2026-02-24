package frc.robot.subsystems;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LeftIntake extends SubsystemBase 
{   
    private static final int CONFIG_RETRIES = 5;

    private static final double STATOR_CURRENT_LIMIT_AMPS = 10.0;   // TODO - Adjust as necessary to prevent damage to the motor and mechanism - set low rn for testing purposes
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 10.0;   // TODO - Adjust as necessary to prevent damage to the motor and mechanism - set low rn for testing purposes
    private static final double MM_CRUISE_RPS = 5.0;
    private static final double MM_ACCEL_RPS2 = 5.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 0; // TOOD - Set this for the intake sprot box
    //PID - TODO tune
    // PIVOT PID
    private static final double SLOT0_KS = 0.0;
    private static final double SLOT0_KV = 0.0;
    private static final double SLOT0_KP = 10.0;
    private static final double SLOT0_KI = 0.0;
    private static final double SLOT0_KD = 1.0;
    // Rollor PID
    private static final double SLOT1_KS = 0.0;
    private static final double SLOT1_KV = 0.0;
    private static final double SLOT1_KP = 10.0;
    private static final double SLOT1_KI = 0.0;
    private static final double SLOT1_KD = 1.0;


    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;

    private double currentAngleSetPoint = 0.0;

    private final TalonFX intake_pivot_motor;
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private static final double SPEED_TOLERANCE_RPS = 0.5;
    private static final double MAX_INTAKE_SPEED_RPS = 40.0; // TODO - tune
    private static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

    private final TalonFX intake_roller_motor;
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
    private double currentSpeedSetpointRps = 0.0;


  public LeftIntake()
  {
    intake_pivot_motor = new TalonFX(Constants.CAN_ID.INTAKE_LEFT_ROTATE);//need to set CAN ID might have to change how it is set because of different motors
        configurePivotMotor(); //TODO Need pass throughs
    intake_roller_motor = new TalonFX(Constants.CAN_ID.INTAKE_LEFT);//need to set CAN ID might have to change how it is set because of different motors
        configureSpinMotor(); //TODO Need pass throughs
  }
 
  public void zeroEncoder() 
  {
     intake_pivot_motor.setPosition(0.0);
  }

    // pivot code

  public void setAngle(double angle)
  {
     intake_pivot_motor.setControl(motionMagicVoltage.withPosition(angle));
        currentAngleSetPoint = angle; // Update the current angle preset for alignment checks
        SmartDashboard.putNumber("LEFT Intake Set Point", angle);
  }

  public double getAngle()
  {
     return intake_pivot_motor.getPosition().getValueAsDouble();
  }

  public boolean isAligned()
    {
        return Math.abs(getAngle() - currentAngleSetPoint) <= Constants.Intake.AngleToleranceDegrees; //TODO TUNE
    }
    // Code for rollers
  public void setSpeed(double speedRps) {
        currentSpeedSetpointRps = speedRps;
        intake_roller_motor.setControl(velocityRequest.withVelocity(currentSpeedSetpointRps));
        SmartDashboard.putNumber("LEFT Intake LEFT Speed Setpoint RPS", currentSpeedSetpointRps);
    }

    
    public void runFeed(double percentOutput) {
        double clamped = MathUtil.clamp(percentOutput, -1.0, 1.0);
        setSpeed(clamped * MAX_INTAKE_SPEED_RPS);
    }

    public double getSpeed() {
        return intake_roller_motor.getVelocity().getValueAsDouble();
    }


    public boolean isAtSpeed(double toleranceRps) {
        return Math.abs(getSpeed() - currentSpeedSetpointRps) <= toleranceRps;
    }

    public void stop() {
        currentSpeedSetpointRps = 0.0;
        intake_roller_motor.stopMotor();
    }


    // TODO Add second congfig. for spin motors
  private void configurePivotMotor()
    {
        TalonFXConfiguration IntakePivotConfigs = new TalonFXConfiguration()
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

        IntakePivotConfigs.Slot0.kS = SLOT0_KS;
        IntakePivotConfigs.Slot0.kV = SLOT0_KV;
        IntakePivotConfigs.Slot0.kP = SLOT0_KP;
        IntakePivotConfigs.Slot0.kI = SLOT0_KI;
        IntakePivotConfigs.Slot0.kD = SLOT0_KD;
        IntakePivotConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;// might need to tweek this
        IntakePivotConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = intake_pivot_motor.getConfigurator().apply(IntakePivotConfigs);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.out.println("Could not apply LEFT pivot intake configs, error code: " + status);
        }
    }
    private void configureSpinMotor()
    {
        TalonFXConfiguration IntakeSpinConfigs = new TalonFXConfiguration()
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

        IntakeSpinConfigs.Slot0.kS = SLOT1_KS;
        IntakeSpinConfigs.Slot0.kV = SLOT1_KV;
        IntakeSpinConfigs.Slot0.kP = SLOT1_KP;
        IntakeSpinConfigs.Slot0.kI = SLOT1_KI;
        IntakeSpinConfigs.Slot0.kD = SLOT1_KD;
        IntakeSpinConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;// might need to tweek this
        IntakeSpinConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = intake_roller_motor.getConfigurator().apply(IntakeSpinConfigs);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.out.println("Could not apply LEFT spin intake configs, error code: " + status);
        }
    }
}

