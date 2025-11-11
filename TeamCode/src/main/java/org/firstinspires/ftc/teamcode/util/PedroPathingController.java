package org.firstinspires.ftc.teamcode.util;

/**
 * Handles robot localization, pose updates, and path calculation for PedroPathing auto routines.
 */
public class PedroPathingController {

    private Pose2d currentPose;    // Current estimated pose of the robot

    public PedroPathingController(Pose2d initialPose) {
        this.currentPose = new Pose2d(initialPose);
    }

    public PedroPathingController() {
        this(new Pose2d(0, 0, 0));
    }

    /**
     * Update the robot's current odometry pose (used in Drivetrain.periodic and localization).
     */
    public void setCurrentPose(Pose2d pose) {
        currentPose = new Pose2d(pose);
    }

    /**
     * Returns the current robot estimated position (used for field-centric and autonomous).
     */
    public Pose2d getCurrentPose() {
        return new Pose2d(currentPose);
    }

    /**
     * Main path following API for drivetrain.
     * Calculates the next movement (holonomic powers: x, y, heading) needed
     * to drive toward the given target. This is where Bézier/centroid PID logic would go.
     *
     * @param start Current robot pose
     * @param target Target pose to reach (field coordinate)
     * @param velocity Desired movement velocity
     * @return Pose2d whose x,y,heading fields represent desired drivetrain powers at this instant
     */
    public Pose2d calculate(Pose2d start, Pose2d target, double velocity) {
        // Simple proportional path following -- replace with Bézier, PID, etc for advanced routines
        double kP = 0.05;  // Path proportional gain, tune as needed

        double dx = target.getX() - start.getX();
        double dy = target.getY() - start.getY();
        double dHeading = target.getHeading() - start.getHeading();

        // For demo, normalize output to velocity
        double distance = Math.hypot(dx, dy);
        double scale = distance > 0.01 ? Math.min(velocity / Math.max(distance, 1e-5), 1.0) : 0.0;

        double xPower = kP * dx * scale;
        double yPower = kP * dy * scale;
        double headingPower = kP * dHeading;

        // .plus(...) updates heading for next iteration, if needed. Here, output is motor powers.
        return new Pose2d(xPower, yPower, headingPower);
    }
}
