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
public class Blue extends OpMode {
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

    public static double xfront_1 = 63, yfront_1 = 15, hfront_1 = 5;
    public static double xfront_2 = -5, yfront_2 = - 63, hfront_2 = 90;
    public static double xfront_5 = -55, yfront_5 = 60, hfront_5 = 90;
    public static double xfront_6 = 65, yfront_6 = -18, hfront_6 = 90;


    public static double xleft_1 = 65, yleft_1 = 8, hleft_1 = 90;
    public static double xleft_2 = -5, yleft_2 = -58, hleft_2 = 90;
    public static double xleft_5 = -78, yleft_5 = 58, hleft_5 = 90;
    public static double xleft_6 = 100, yleft_6 = -20, hleft_6 = 90;

    public static double xright_1 = 40, yright_1 = -18, hright_1 = 0;
    public static double xright_2 = 20, yright_2 = -20, hright_2 = 90;


    public static double x3 = 65, y3 = 15, h3 = 90;
    public static double x4 = 0, y4 = 200, h4 = 90;

    private double translationError = 0;
    private double headingError = 0;
    private int framecounter = 0;

    public static double translationTolerance = 5.5;
    public static double headingTolerance = 5;


    public int zona;


    @Override
    public void init() {
        RobotHardware.pipelineColour = PipelineStanga.team.albastru;

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
                    if (zona == 1 || zona == 2) nextAutoInstruction = AutoInstruction.INSTRUCTION_2;
                    else nextAutoInstruction = AutoInstruction.INSTRUCTION_11;
                    waitingTime = 0.5; timer.reset();
                }

                break;

            case INSTRUCTION_2:

                switch (zona) {
                    case 1:
                        x = xleft_2; y = yleft_2; h = hleft_2;
                        break;
                    case 2:
                        x = xfront_2; y = yfront_2; h = hfront_2;
                        break;
                    case 3:
                        x = xright_2; y = yright_2; h = hright_2;
                }

                ActiveIntake.servoState = ActiveIntake.ServoState.PIXEL_5;
                Virtual4Bar.v4bState = Virtual4Bar.V4BState.INIT;
                Claw.clawState = Claw.ClawState.INIT;
                Claw.stReleaseState = Claw.StReleaseState.INIT;
                ndReleaseState = Claw.NdReleaseState.INIT;

                calculateErrors();

                if (reachedDest()) {

                    autoInstruction = AutoInstruction.WAITING;
                    nextAutoInstruction = AutoInstruction.INSTRUCTION_3;
                    waitingTime = 1; timer.reset();
                }

                break;

            case INSTRUCTION_3:

                x = 0; y = y3; h = h3;
                ActiveIntake.servoState = ActiveIntake.ServoState.RAISE;

                calculateErrors();

                if (reachedDest()) {

                    autoInstruction = AutoInstruction.WAITING;
                    nextAutoInstruction = AutoInstruction.INSTRUCTION_4;
                    waitingTime = 0.3;
                    timer.reset();
                }
                break;

            case INSTRUCTION_4:

                x = x3; y = 0; h = h3;
                ActiveIntake.servoState = ActiveIntake.ServoState.INTAKE;

                calculateErrors();

                if (reachedDest()) {
                    autoInstruction = AutoInstruction.WAITING;
                    nextAutoInstruction = AutoInstruction.INSTRUCTION_4_1;
                    waitingTime = 0.3; timer.reset();
                }

                break;

            case INSTRUCTION_4_1:

                x = x4; y = y4 / 2; h = h4 + 1;
                ActiveIntake.servoState = ActiveIntake.ServoState.OUTTAKE;

                calculateErrors();

                if (reachedDest()) {
                    autoInstruction = AutoInstruction.WAITING;
                    nextAutoInstruction = AutoInstruction.INSTRUCTION_5;
                    waitingTime = 0; timer.reset();
                }

                break;

            case INSTRUCTION_5:

                x = x4; y = y4 / 2; h = h4;
                ActiveIntake.servoState = ActiveIntake.ServoState.OUTTAKE;

                calculateErrors();

                if (reachedDest()) {
                    ActiveIntake.servoState = ActiveIntake.ServoState.RAISE;
                    //Extendo.extendoState = Extendo.ExtendoState.READY_FOR_TRANSFER;

                    autoInstruction = AutoInstruction.WAITING;
                    nextAutoInstruction = AutoInstruction.INSTRUCTION_6;
                    waitingTime = 2; timer.reset();
                }

                break;

            case INSTRUCTION_6:
                Transfer.initiateTransfer = true;

                calculateErrors();

                autoInstruction = AutoInstruction.WAITING;
                nextAutoInstruction = AutoInstruction.INSTRUCTION_7;
                waitingTime = 2; timer.reset();
                break;

            case INSTRUCTION_7:

                switch (zona) {
                    case 1:
                        x = xleft_5; y = yleft_5; h = hleft_5;
                        break;

                    case 2:
                        x = xfront_5; y = yfront_5; h = hfront_5;
                        break;
                }

                Virtual4Bar.v4bState = Virtual4Bar.V4BState.RELEASE;
                Claw.clawState = Claw.ClawState.RELEASE;
                ndReleaseState = Claw.NdReleaseState.RELEASE_CLOSED;
                Lift.liftState = Lift.LiftState.AUTO_1;

                calculateErrors();

                if (reachedDest()) {
                    autoInstruction = AutoInstruction.WAITING;
                    nextAutoInstruction = AutoInstruction.INSTRUCTION_8;
                    waitingTime = 1; timer.reset();
                }

                break;

            case INSTRUCTION_8:

                Claw.stReleaseState = Claw.StReleaseState.RELEASE_OPEN;

                autoInstruction = AutoInstruction.WAITING;
                nextAutoInstruction = AutoInstruction.INSTRUCTION_9;
                waitingTime = 1; timer.reset();

                break;

            case INSTRUCTION_9:

                Claw.stReleaseState = Claw.StReleaseState.RELEASE_MID_OPEN;
                ndReleaseState = Claw.NdReleaseState.RELEASE_OPEN;

                autoInstruction = AutoInstruction.WAITING;
                nextAutoInstruction = AutoInstruction.INSTRUCTION_11;
                waitingTime = 1; timer.reset();

                break;

            case INSTRUCTION_10:

                switch (zona) {
                    case 1:
                        x = xleft_6; y = yleft_6; h = hleft_6;
                        break;

                    case 2:
                        x = xfront_6; y = yfront_6; h = hfront_6;
                        break;
                }

                calculateErrors();

                if (reachedDest()) {
                    autoInstruction = AutoInstruction.WAITING;
                    nextAutoInstruction = AutoInstruction.INSTRUCTION_11;
                    waitingTime = 0.3; timer.reset();
                }

                break;

            case INSTRUCTION_11:

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
