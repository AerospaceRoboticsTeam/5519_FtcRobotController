package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.PedroPathingController;
import org.firstinspires.ftc.teamcode.util.Pose2d;

public class AdvancedDrivetrain extends SubsystemBase {
    // MotorEx for FTCLib and easy integration with odometry
    private final MotorEx frontLeft, frontRight, backLeft, backRight;
    // IMU for heading
    private final IMU imu;
    // PedroPathing controller for autonomous path following
    private final PedroPathingController pathController;
    // Panels telemetry manager
    private final TelemetryManager panelsTelemetry;
    // Internal pose estimate
    private Pose2d currentPose = new Pose2d(0,0,0);
    // Tuning variables
    private double boost = 1.0;
    private double xSensitivity = 1.0, ySensitivity = 1.0, rxSensitivity = 1.0;

    public AdvancedDrivetrain(HardwareMap hardwareMap, PedroPathingController pathController) {
        // Setup motors
        frontLeft = new MotorEx(hardwareMap, "left_front_mtr");
        frontRight = new MotorEx(hardwareMap, "right_front_mtr");
        backLeft = new MotorEx(hardwareMap, "left_back_mtr");
        backRight = new MotorEx(hardwareMap, "right_back_mtr");

        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Panels telemetry
        panelsTelemetry = Panels.getTelemetry();

        // PedroPathing controller
        this.pathController = pathController;
    }

    public void setBoost(double boost) {
        this.boost = Math.max(0.0, Math.min(1.0, boost));
    }

    public void setXSensitivity(double sensitiv) { this.xSensitivity = sensitiv; }
    public void setYSensitivity(double sensitiv) { this.ySensitivity = sensitiv; }
    public void setRxSensitivity(double sensitiv) { this.rxSensitivity = sensitiv; }

    /**
     * Driver control: field-centric mecanum
     */
    public void drive(double x, double y, double rx) {
        double heading = getHeading();
        // Field-centric transform
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double[] powers = new double[]{
                rotY + rotX + rx,
                rotY - rotX + rx,
                rotY - rotX - rx,
                rotY + rotX - rx
        };

        double max = 1.0;
        for (double p : powers) max = Math.max(max, Math.abs(p));
        for (int i = 0; i < powers.length; i++) powers[i] = powers[i] / max * boost;

        frontLeft.set(powers[0]);
        backLeft.set(powers[1]);
        frontRight.set(powers[2]);
        backRight.set(powers[3]);
    }
    /**
     * Autonomous control via PedroPathing
     */
    public void followPath(Pose2d targetPose, double velocity) {
        // Calculate next movement using PedroPathing
        Pose2d movement = pathController.calculate(currentPose, targetPose, velocity);
        drive(movement.getX(), movement.getY(), movement.getHeading());
    }

    /**
     * Odometry update -- integrate encoder and IMU readings
     */
    public void updateOdometry() {
        currentPose = pathController.getCurrentPose();
    }

    /**
     * Panels Telemetry for Debugging
     */
    @Override
    public void periodic() {
        // Update odometry
        updateOdometry();
        // Panels telemetry for live robot status
        // TODO: Refine for reducing chances of bugs, suggested by the IDE
        panelsTelemetry.debug(
                String.format("Pose X: %.2f", currentPose.getX()),
                String.format("Pose Y: %.2f", currentPose.getY()),
                String.format("Heading: %.2f deg", Math.toDegrees(currentPose.getHeading())),
                String.format("LF Power: %.2f", frontLeft.get()),
                String.format("LB Power: %.2f", backLeft.get()),
                String.format("RF Power: %.2f", frontRight.get()),
                String.format("RB Power: %.2f", backRight.get())
        );
        panelsTelemetry.update();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public Pose2d getCurrentPose() { return currentPose; }

    // Add getMotor methods as needed later...
}
