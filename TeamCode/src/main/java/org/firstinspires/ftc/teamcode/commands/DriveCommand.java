package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.AdvancedDrivetrain;
import org.firstinspires.ftc.teamcode.util.Pose2d;

import java.util.function.DoubleSupplier;

/**
 * Default drive command - field-centric mecanum control
 * Uses double suppliers for real-time gamepad input
 */
public class DriveCommand extends CommandBase {
    private final AdvancedDrivetrain drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rxSupplier;

    public DriveCommand(AdvancedDrivetrain drivetrain,
                        DoubleSupplier xSupplier,
                        DoubleSupplier ySupplier,
                        DoubleSupplier rxSupplier) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rxSupplier = rxSupplier;
        addRequirements(drivetrain);
    }

    /**
     * Convenience constructor using GamepadEx
     */
    public DriveCommand(AdvancedDrivetrain drivetrain, GamepadEx gamepad) {
        this(drivetrain,
                () -> -gamepad.getLeftX(),   // Strafe
                () -> -gamepad.getLeftY(),   // Forward/back
                () -> -gamepad.getRightX()); // Rotation
    }

    @Override
    public void execute() {
        drivetrain.drive(xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                rxSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}


/**
 * Drive to specific pose using PedroPathing
 * Finishes when target is reached within tolerance
 */
class DriveToPoseCommand extends CommandBase {
    private final AdvancedDrivetrain drivetrain;
    private final Pose2d targetPose;
    private final double velocity;
    private final double positionTolerance;
    private final double headingTolerance;
    private final long timeout;
    private long startTime;

    public DriveToPoseCommand(AdvancedDrivetrain drivetrain,
                              Pose2d targetPose,
                              double velocity,
                              double positionTolerance,
                              double headingTolerance,
                              double timeoutSeconds) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.velocity = velocity;
        this.positionTolerance = positionTolerance;
        this.headingTolerance = headingTolerance;
        this.timeout = (long)(timeoutSeconds * 1000);
        addRequirements(drivetrain);
    }

    /**
     * Standard constructor with default tolerances
     */
    public DriveToPoseCommand(AdvancedDrivetrain drivetrain, Pose2d targetPose, double velocity) {
        this(drivetrain, targetPose, velocity, 2.0, Math.toRadians(5), 5.0);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        drivetrain.followPath(targetPose, velocity);
    }

    @Override
    public boolean isFinished() {
        Pose2d current = drivetrain.getCurrentPose();

        double dx = targetPose.getX() - current.getX();
        double dy = targetPose.getY() - current.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        double headingError = Math.abs(targetPose.getHeading() - current.getHeading());

        boolean atTarget = distance < positionTolerance && headingError < headingTolerance;
        boolean timedOut = (System.currentTimeMillis() - startTime) > timeout;

        return atTarget || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drivetrain.drive(0, 0, 0); // Stop motors
        }
    }
}


/**
 * Reset IMU heading to zero
 * Useful for field-centric driving calibration
 */
class ResetHeadingCommand extends CommandBase {
    private final AdvancedDrivetrain drivetrain;

    public ResetHeadingCommand(AdvancedDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset IMU - you'll need to add this method to AdvancedDrivetrain:
        // public void resetHeading() { imu.resetYaw(); }
        // drivetrain.resetHeading();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}


/**
 * Stop all drivetrain motors immediately
 */
class StopDriveCommand extends CommandBase {
    private final AdvancedDrivetrain drivetrain;

    public StopDriveCommand(AdvancedDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}


/**
 * Set boost/speed multiplier
 */
class SetBoostCommand extends CommandBase {
    private final AdvancedDrivetrain drivetrain;
    private final double boost;

    public SetBoostCommand(AdvancedDrivetrain drivetrain, double boost) {
        this.drivetrain = drivetrain;
        this.boost = boost;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setBoost(boost);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}


/**
 * Rotate to specific heading while maintaining position
 */
class RotateToHeadingCommand extends CommandBase {
    private final AdvancedDrivetrain drivetrain;
    private final double targetHeading;
    private final double rotationSpeed;
    private final double tolerance;
    private final long timeout;
    private long startTime;

    public RotateToHeadingCommand(AdvancedDrivetrain drivetrain,
                                  double targetHeading,
                                  double rotationSpeed,
                                  double toleranceDegrees,
                                  double timeoutSeconds) {
        this.drivetrain = drivetrain;
        this.targetHeading = targetHeading;
        this.rotationSpeed = rotationSpeed;
        this.tolerance = Math.toRadians(toleranceDegrees);
        this.timeout = (long)(timeoutSeconds * 1000);
        addRequirements(drivetrain);
    }

    public RotateToHeadingCommand(AdvancedDrivetrain drivetrain, double targetHeading) {
        this(drivetrain, targetHeading, 0.5, 2.0, 3.0);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double currentHeading = drivetrain.getHeading();
        double error = targetHeading - currentHeading;

        // Normalize error to [-π, π]
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        double rotationPower = Math.signum(error) * rotationSpeed;
        drivetrain.drive(0, 0, rotationPower);
    }

    @Override
    public boolean isFinished() {
        double error = Math.abs(targetHeading - drivetrain.getHeading());
        boolean atTarget = error < tolerance;
        boolean timedOut = (System.currentTimeMillis() - startTime) > timeout;
        return atTarget || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
    }
}


/**
 * Drive straight for a distance in field coordinates
 */
class DriveStraightCommand extends CommandBase {
    private final AdvancedDrivetrain drivetrain;
    private final double distance;
    private final double velocity;
    private final double heading; // Direction to drive
    private Pose2d startPose;
    private Pose2d targetPose;

    public DriveStraightCommand(AdvancedDrivetrain drivetrain,
                                double distance,
                                double velocity,
                                double heading) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        this.velocity = velocity;
        this.heading = heading;
        addRequirements(drivetrain);
    }

    /**
     * Drive forward (current heading)
     */
    public DriveStraightCommand(AdvancedDrivetrain drivetrain, double distance, double velocity) {
        this(drivetrain, distance, velocity, drivetrain.getHeading());
    }

    @Override
    public void initialize() {
        startPose = drivetrain.getCurrentPose();

        double targetX = startPose.getX() + distance * Math.cos(heading);
        double targetY = startPose.getY() + distance * Math.sin(heading);

        targetPose = new Pose2d(targetX, targetY, startPose.getHeading());
    }

    @Override
    public void execute() {
        drivetrain.followPath(targetPose, velocity);
    }

    @Override
    public boolean isFinished() {
        Pose2d current = drivetrain.getCurrentPose();
        double dx = targetPose.getX() - current.getX();
        double dy = targetPose.getY() - current.getY();
        double distanceRemaining = Math.sqrt(dx * dx + dy * dy);

        return distanceRemaining < 2.0; // 2 inch tolerance
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
    }
}


/**
 * Strafe (sideways movement) for a distance
 */
class StrafeCommand extends CommandBase {
    private final AdvancedDrivetrain drivetrain;
    private final double distance;
    private final double velocity;
    private Pose2d startPose;
    private Pose2d targetPose;

    public StrafeCommand(AdvancedDrivetrain drivetrain, double distance, double velocity) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        this.velocity = velocity;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startPose = drivetrain.getCurrentPose();
        double heading = drivetrain.getHeading();

        // Perpendicular to heading
        double strafeHeading = heading + Math.PI / 2;
        double targetX = startPose.getX() + distance * Math.cos(strafeHeading);
        double targetY = startPose.getY() + distance * Math.sin(strafeHeading);

        targetPose = new Pose2d(targetX, targetY, startPose.getHeading());
    }

    @Override
    public void execute() {
        drivetrain.followPath(targetPose, velocity);
    }

    @Override
    public boolean isFinished() {
        Pose2d current = drivetrain.getCurrentPose();
        double dx = targetPose.getX() - current.getX();
        double dy = targetPose.getY() - current.getY();
        double distanceRemaining = Math.sqrt(dx * dx + dy * dy);

        return distanceRemaining < 2.0;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
    }
}