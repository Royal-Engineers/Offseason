package org.firstinspires.ftc.opmodes;

import static org.firstinspires.ftc.objects.outtake.Claw.ndReleaseState;
import static org.firstinspires.ftc.robot.StaticVariables.robotX;
import static org.firstinspires.ftc.robot.StaticVariables.robotY;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous
public class Red extends OpMode {
    public enum AutoInstruction {

        WAITING,

        INSTRUCTION_1,
        INSTRUCTION_2,
        INSTRUCTION_3,
        INSTRUCTION_4, INSTRUCTION_4_1,
        INSTRUCTION_5,
        INSTRUCTION_6,
        INSTRUCTION_7,
        INSTRUCTION_8,
        INSTRUCTION_9,
        INSTRUCTION_10,
        INSTRUCTION_11,
        INSTRUCTION_12;

    }
    public AutoInstruction autoInstruction, nextAutoInstruction;

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
    private double waitingTime = 0;

    public static double x = 0, y = 0, h = 0;
    private double clx = 0, cly = 0, clh = 0;

    public static double xfront_1 = 63, yfront_1 = 0, hfront_1 = 5;

    public static double xleft_1 = 40, yleft_1 = 18, hleft_1 = 0;

    public static double xright_1 = 65, yright_1 = -5, hright_1 = 270;

    private double translationError = 0;
    private double headingError = 0;
    private int framecounter = 0;

    public static double translationTolerance = 5.5;
    public static double headingTolerance = 5;

    public int zona;


    @Override
    public void init() {
        RobotHardware.pipelineColour = PipelineStanga.team.rosu;

        autoInstruction = AutoInstruction.INSTRUCTION_1;

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

        Claw.stReleaseState = Claw.StReleaseState.CLOSED;
        autoInstruction = AutoInstruction.INSTRUCTION_1;
    }

    @Override
    public void init_loop() {
        zona = PipelineStanga.region_of_interest;

        switch (zona) {

            case 1:

                x = xleft_1; y = yleft_1; h = hleft_1;
                break;

            case 2:

                x = xfront_1; y = yfront_1; h = hfront_1;
                break;

            case 3:

                x = xright_1; y = yright_1; h = hright_1;
                break;

        }
    }

    private void updateObjects() {
        v4b.update();
        claw.update();
        lift.update();
        extendo.update();
        activeIntake.update();
    }

    private void calculateErrors() {
        translationError = Math.sqrt(Math.pow(x- StaticVariables.robotX, 2) + Math.pow(y-StaticVariables.robotY, 2));
        headingError = h - Math.toDegrees(StaticVariables.robotTheta);
        if (headingError > 180) headingError -= 360;
        else if (headingError < -180) headingError += 360;
    }

    private boolean reachedDest () {
        if(Math.abs(headingError) < headingTolerance) framecounter++;
        else framecounter = 0;
        if(Math.abs(translationError) < translationTolerance && Math.abs(headingError) < headingTolerance && framecounter >= 30 )
            return true;
        return false;
    }

    @Override
    public void loop() {

        if (x != clx || y != cly || h !=clh) {
            robotX = robotX - clx; robotY = robotY - cly;
            linearPath = new LinearPath(x, y, h);
        }
        clx = x; cly = y; clh = h;

        if (autoInstruction == AutoInstruction.WAITING && timer.seconds() > waitingTime) {
            autoInstruction = nextAutoInstruction;
        }

        switch (autoInstruction) {

            case INSTRUCTION_1:

                ndReleaseState = Claw.NdReleaseState.TRANSFER;
                Virtual4Bar.v4bState = Virtual4Bar.V4BState.RELEASE;
                Claw.clawState = Claw.ClawState.RELEASE_AUTO;

                calculateErrors();

                if (reachedDest()) {
                    Claw.stReleaseState = Claw.StReleaseState.TRANSFER;

                    autoInstruction = AutoInstruction.WAITING;
                    nextAutoInstruction = AutoInstruction.INSTRUCTION_2;
                    waitingTime = 0.5; timer.reset();
                }

                break;

            case INSTRUCTION_2:

                Extendo.extendoState = Extendo.ExtendoState.AUTO_FINISH;
                Lift.liftState = Lift.LiftState.AUTO_FINISH;

                telemetry.addLine("BELLINGHAM");

                break;

        }



        odometry.update(); calculateErrors(); linearPath.update();
        driveSubsystem.updateAuto(linearPath.getOutput_velocity_x(), linearPath.getOutput_velocity_y(), linearPath.getOutput_velocity_theta());

        updateObjects();
        commands.update();
        robot.update();

        telemetry.update();
    }
}
