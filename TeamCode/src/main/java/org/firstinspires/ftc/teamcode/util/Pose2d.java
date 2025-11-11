package org.firstinspires.ftc.teamcode.util;

/**
 * Represents a 2D robot pose (x, y position + heading in radians).
 */
public class Pose2d {
    private double x;
    private double y;
    private double heading; // In radians

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d() {
        this(0.0, 0.0, 0.0);
    }

    // Copy constructor
    public Pose2d(Pose2d other) {
        this.x = other.x;
        this.y = other.y;
        this.heading = other.heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    // Calculate Euclidean distance to another pose or point
    public double distanceTo(Pose2d other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    // Add values from another pose (useful for odometry update)
    public Pose2d plus(Pose2d other) {
        return new Pose2d(this.x + other.x, this.y + other.y, this.heading + other.heading);
    }

    // Subtract values from another pose
    public Pose2d minus(Pose2d other) {
        return new Pose2d(this.x - other.x, this.y - other.y, this.heading - other.heading);
    }

    // String representation for telemetry
    @Override
    public String toString() {
        return String.format("(%.2f, %.2f, %.2f rad)", x, y, heading);
    }
}
