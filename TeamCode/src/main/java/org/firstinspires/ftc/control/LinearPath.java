package org.firstinspires.ftc.control;

import static org.firstinspires.ftc.robot.StaticVariables.chassisAcceleration;
import static org.firstinspires.ftc.robot.StaticVariables.chassisVelocity;
import static org.firstinspires.ftc.robot.StaticVariables.robotTheta;
import static org.firstinspires.ftc.robot.StaticVariables.robotX;
import static org.firstinspires.ftc.robot.StaticVariables.robotY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.objects.chassis.DriveSubsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LinearPath {

    FtcDashboard dashboard= FtcDashboard.getInstance();
    Telemetry dashboardTelemetry=dashboard.getTelemetry();

    private double destination_x, destination_y, destination_theta;
    private double distance, alpha;

    private double target_x, target_y, output_velocity_x, output_velocity_y, output_velocity_theta, velocity, correction_velocity_x, correction_velocity_y;
    private double area, height, beta;
    public static double Kt = 2.5, Kn = 2, K = 12, Ka = 0.5, KI = 0, integralSum;
    public static double P = 1.4, I = 0, D = 0.2, F = 0;

    private double error;

    private MotionProfile motionProfile;
    private PIDF pidf;

    public LinearPath(double destination_x, double destination_y, double destination_theta) {
        this.destination_x = destination_x;
        this.destination_y = destination_y;
        this.destination_theta = Math.toRadians(destination_theta);

        distance = Math.sqrt(Math.pow(destination_x, 2) + Math.pow(destination_y, 2));
        alpha = Math.atan2(destination_y, destination_x); if (alpha < 0) alpha += 2 * Math.PI;
        beta = alpha - Math.PI / 2; if (beta < 0) beta += 2 * Math.PI;

        motionProfile = new MotionProfile(chassisVelocity, chassisAcceleration, distance);
        integralSum = 0;

        pidf = new PIDF();
        pidf.resetReference();
        pidf.setTargetSpeed(1);
        pidf.setCoefficients(P, I, D, F);
    }

    private double getDifference(double theta1, double theta2) {
        double dif = theta1 - theta2;

        if (dif > Math.PI) dif -= 2 * Math.PI;
        else if (dif < -Math.PI) dif += 2 * Math.PI;

        return dif;
    }

    public void update() {
        motionProfile.update();
        pidf.setCoefficients(P, I, D, F);

        double x1 = 0, y1 = 0, x2 = destination_x, y2 = destination_y, x3 = robotX, y3 = robotY;

        area = 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
        if (distance != 0) height = 2 * area / distance;
        else height = 0;

        correction_velocity_x = Math.cos(beta) * height;
        correction_velocity_y = Math.sin(beta) * height;

        target_x = Math.cos(alpha) * motionProfile.getPosition();
        target_y = Math.sin(alpha) * motionProfile.getPosition();

        output_velocity_x = Math.cos(alpha) * motionProfile.getVelocity() * Kt + Math.cos(alpha) * motionProfile.getAcceleration() * Ka + correction_velocity_x * Kn + (target_x - robotX) * K + Math.cos(alpha) * integralSum * KI;
        output_velocity_y = Math.sin(alpha) * motionProfile.getVelocity() * Kt + Math.sin(alpha) * motionProfile.getAcceleration() * Ka + correction_velocity_y * Kn + (target_y - robotY) * K + Math.sin(alpha) * integralSum * KI;
        integralSum = integralSum + Math.sqrt(Math.pow(target_x - robotX, 2) + Math.pow(target_y - robotY, 2));

        output_velocity_x = output_velocity_x / chassisVelocity;
        output_velocity_y = output_velocity_y / chassisVelocity;

        velocity = Math.sqrt(Math.pow(output_velocity_x, 2) + Math.pow(output_velocity_y, 2));
        if (velocity > 1) {
            output_velocity_x /= velocity;
            output_velocity_y /= velocity;
        }

        error = getDifference(destination_theta, robotTheta);
        if (Math.toDegrees(Math.abs(error)) < 2) output_velocity_theta = 0;
        else output_velocity_theta = Math.min(pidf.getOutput(error), 1);

        dashboardTelemetry.addData("output x", output_velocity_x);
        dashboardTelemetry.addData("output y", output_velocity_y);
        //dashboardTelemetry.addData("output theta", output_velocity_theta);

        dashboardTelemetry.addData("Error", Math.sqrt(Math.pow(target_x - robotX, 2) + Math.pow(target_y - robotY, 2)));
        dashboardTelemetry.addData("ErrorTheta", Math.toDegrees(error));


        dashboardTelemetry.addData("X", robotX);
        dashboardTelemetry.addData("Y", robotY);
        dashboardTelemetry.addData("THETA", Math.toDegrees(robotTheta));

        dashboardTelemetry.update();
    }

    public double getOutput_velocity_x() {
        return -output_velocity_y;
    }

    public double getOutput_velocity_y() {
        return output_velocity_x;
    }
    public double getOutput_velocity_theta() {
        return output_velocity_theta;
    }
}
