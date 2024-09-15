package org.firstinspires.ftc.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.commands.Commands;
import org.firstinspires.ftc.commands.Transfer;
import org.firstinspires.ftc.control.LinearPath;
import org.firstinspires.ftc.objects.chassis.DriveSubsystem;
import org.firstinspires.ftc.objects.chassis.Odometry;
import org.firstinspires.ftc.objects.intake.ActiveIntake;
import org.firstinspires.ftc.objects.intake.Extendo;
import org.firstinspires.ftc.objects.outtake.Claw;
import org.firstinspires.ftc.objects.outtake.Lift;
import org.firstinspires.ftc.objects.outtake.Virtual4Bar;
import org.firstinspires.ftc.pipelinuri.PipelineStanga;
import org.firstinspires.ftc.robot.RobotHardware;
import org.firstinspires.ftc.robot.StaticVariables;

@Autonomous
public class TestAuto extends OpMode {
    private RobotHardware robot;

    private DriveSubsystem driveSubsystem;
    private Odometry odometry;
    private LinearPath linearPath;

    private Virtual4Bar v4b;
    private Claw claw;
    private Lift lift;

    private Extendo extendo;
    private ActiveIntake activeIntake;
    private Commands commands;

    private ElapsedTime timer = new ElapsedTime();

    private boolean lol = true;


    @Override
    public void init() {
        StaticVariables.init(hardwareMap, telemetry, gamepad1, gamepad2);

        robot = new RobotHardware();
        robot.init();

        driveSubsystem = new DriveSubsystem(robot);
        odometry = new Odometry(robot);

        v4b = new Virtual4Bar(robot);
        claw = new Claw(robot);
        lift = new Lift(robot);

        activeIntake = new ActiveIntake(robot);
        extendo = new Extendo(robot);
        commands = new Commands();
    }

    private void updateObjects() {
        v4b.update();
        claw.update();
        lift.update();
        extendo.update();
        activeIntake.update();
    }

    @Override
    public void loop() {
        if (lol) {
            Transfer.initiateTransfer = true;
            lol = false;
        }

        updateObjects();
        commands.update();
        robot.update();

        telemetry.update();
    }
}
