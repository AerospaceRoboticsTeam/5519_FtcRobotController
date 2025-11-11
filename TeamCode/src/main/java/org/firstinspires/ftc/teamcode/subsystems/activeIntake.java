package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Archive.Libs.AR.AR_PIDController;
public class activeIntake extends SubsystemBase {
    private DcMotor intakeMotor;
    private AR_PIDController pidController;
    private double ticksPerDegree;
    private double targetAngle = 0.0;
    private boolean holdPosition = false;

    // Tunable parameters - accessed directly from OpMode
    public double intakePower = 1.0;
    public double outtakePower = -0.8;
    public double manualTargetAngle = 0.0;
    public double tuneKP = 0.01;
    public double tuneKI = 0.0;
    public double tuneKD = 0.0;
    public double tuneKF = 0.0;
    public boolean enablePIDTuning = false;
    public boolean useManualTarget = false;
    // Constructor with fuzzy logic parameter
    public activeIntake(HardwareMap hardwareMap, String motorName, double ticksPerDegree,
                        double kP, double kI, double kD, double kF, boolean fuzzyLogicActive) {
        this.ticksPerDegree = ticksPerDegree;

        // Initialize tuning parameters
        this.tuneKP = kP;
        this.tuneKI = kI;
        this.tuneKD = kD;
        this.tuneKF = kF;

        // Initialize motor
        this.intakeMotor = hardwareMap.dcMotor.get(motorName);
        this.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize PID controller
        this.pidController = new AR_PIDController(null, intakeMotor, motorName, ticksPerDegree, kP, kI, kD, kF, fuzzyLogicActive);
    }

    // Simplified constructor without fuzzy logic
    public activeIntake(HardwareMap hardwareMap, String motorName, double ticksPerDegree,
                        double kP, double kI, double kD, double kF) {
        this(hardwareMap, motorName, ticksPerDegree, kP, kI, kD, kF, false);
    }

    @Override
    public void periodic() {
        // Update PID coefficients if tuning is enabled
        if (enablePIDTuning) {
            updatePIDCoefficients();
        }

        // Use manual target if enabled
        if (useManualTarget) {
            this.targetAngle = manualTargetAngle;
            this.holdPosition = true;
        }

        // Update PID control if holding position
        if (holdPosition) {
            int currentPosition = (int) intakeMotor.getCurrentPosition();
            pidController.loop(targetAngle, currentPosition, 0);
        }
    }

    private void updatePIDCoefficients() {
        // Update PID coefficients dynamically
        // You'll need to add setter methods to your AR_PIDController
        // Example: pidController.setPID(tuneKP, tuneKI, tuneKD, tuneKF);
    }

    // Command-friendly methods
    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
        this.holdPosition = true;
    }

    public void setRawPower(double power) {
        this.holdPosition = false;
        intakeMotor.setPower(power);
    }

    public void intake() {
        setRawPower(intakePower);
    }

    public void outtake() {
        setRawPower(outtakePower);
    }

    public void stop() {
        setRawPower(0.0);
        this.holdPosition = false;
    }

    public void holdCurrentPosition() {
        this.targetAngle = getCurrentAngle();
        this.holdPosition = true;
    }

    // Getters for telemetry and debugging
    public double getCurrentAngle() {
        return intakeMotor.getCurrentPosition() / ticksPerDegree;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getCurrentPosition() {
        return intakeMotor.getCurrentPosition();
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(getCurrentAngle() - targetAngle) < tolerance;
    }

    public DcMotor getMotor() {
        return intakeMotor;
    }

    public AR_PIDController getPIDController() {
        return pidController;
    }

    public boolean isHoldingPosition() {
        return holdPosition;
    }
}