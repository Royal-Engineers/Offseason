package org.firstinspires.ftc.opmodes;

import static org.firstinspires.ftc.robot.StaticVariables.robotTheta;
import static org.firstinspires.ftc.robot.StaticVariables.robotX;
import static org.firstinspires.ftc.robot.StaticVariables.robotY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.commands.Commands;
import org.firstinspires.ftc.control.LinearPath;
import org.firstinspires.ftc.objects.PlaneLauncher;
import org.firstinspires.ftc.objects.chassis.DriveSubsystem;
import org.firstinspires.ftc.objects.chassis.Odometry;
import org.firstinspires.ftc.objects.intake.ActiveIntake;
import org.firstinspires.ftc.objects.intake.Extendo;
import org.firstinspires.ftc.objects.outtake.Claw;
import org.firstinspires.ftc.objects.outtake.Lift;
import org.firstinspires.ftc.objects.outtake.Virtual4Bar;
import org.firstinspires.ftc.robot.RobotHardware;
import org.firstinspires.ftc.robot.StaticVariables;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp(name = "Test")
public class Test extends OpMode {
    private RobotHardware robot;

    private Extendo extendo;
    private ActiveIntake activeIntake;
    private DriveSubsystem driveSubsystem;
    private Odometry odometry;
    private Lift lift;
    private Virtual4Bar v4b;
    private Claw claw;

    FtcDashboard dashboard= FtcDashboard.getInstance();
    Telemetry dashboardTelemetry=dashboard.getTelemetry();

    LinearPath linearPath;

    public static double x = 0, y = 0, theta = 0;
    private double clx = 0, cly = 0, cltheta = 0;


    private void updateObjects() {
        odometry.update();
        extendo.update();
        activeIntake.update();
        v4b.update();
        claw.update();
        lift.update();
    }

    @Override
    public void init() {
        StaticVariables.init(hardwareMap, telemetry, gamepad1, gamepad2);

        robot = new RobotHardware();
        robot.init();

        extendo = new Extendo(robot);
        activeIntake = new ActiveIntake(robot);
        driveSubsystem = new DriveSubsystem(robot);
        odometry = new Odometry(robot);
        lift = new Lift(robot);
        v4b = new Virtual4Bar(robot);
        claw = new Claw(robot);

        linearPath = new LinearPath(x, y, theta);

    }

    @Override
    public void loop() {
        odometry.update();

        dashboardTelemetry.addData("X", robotX);
        dashboardTelemetry.addData("Y", robotY);
        dashboardTelemetry.addData("Theta", robotTheta);

        telemetry.update();
        dashboardTelemetry.update();
    }
}
