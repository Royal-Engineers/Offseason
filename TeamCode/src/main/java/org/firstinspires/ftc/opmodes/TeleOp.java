package org.firstinspires.ftc.opmodes;


import static org.firstinspires.ftc.robot.StaticVariables.robotX;
import static org.firstinspires.ftc.robot.StaticVariables.robotY;

import android.media.Image;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.commands.Commands;
import org.firstinspires.ftc.objects.PlaneLauncher;
import org.firstinspires.ftc.objects.chassis.Odometry;
import org.firstinspires.ftc.objects.intake.ActiveIntake;
import org.firstinspires.ftc.objects.outtake.Claw;
import org.firstinspires.ftc.objects.outtake.Lift;
import org.firstinspires.ftc.objects.outtake.Virtual4Bar;
import org.firstinspires.ftc.robot.RobotHardware;
import org.firstinspires.ftc.objects.intake.Extendo;
import org.firstinspires.ftc.objects.chassis.DriveSubsystem;
import org.firstinspires.ftc.robot.StaticVariables;
import org.openftc.easyopencv.OpenCvPipeline;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends OpMode {
    private RobotHardware robot;

    private Extendo extendo;
    private ActiveIntake activeIntake;
    private DriveSubsystem driveSubsystem;
    private Odometry odometry;
    private Lift lift;
    private Virtual4Bar v4b;
    private Claw claw;
    private PlaneLauncher planeLauncer;

    private Commands commands;

    private ElapsedTime timer = new ElapsedTime();
    private int cnt, lastcnt;

    private void updateObjects() {
        driveSubsystem.update();
        odometry.update();
        extendo.update();
        activeIntake.update();
        v4b.update();
        claw.update();
        lift.update();
        planeLauncer.update();
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
        planeLauncer = new PlaneLauncher(robot);

        commands = new Commands();

        timer.reset(); cnt = 0; lastcnt = 0;


    }

    @Override
    public void loop() {
        commands.update();

        updateObjects();

        robot.update(); cnt++;

        if(timer.seconds() > 1) {
            timer.reset();
            lastcnt = cnt;
            cnt=0;
        }



        telemetry.addData("x: ", robotX);
        telemetry.addData("y: ", robotY);

        telemetry.addData("FPS", lastcnt);
        telemetry.update();
    }
}
