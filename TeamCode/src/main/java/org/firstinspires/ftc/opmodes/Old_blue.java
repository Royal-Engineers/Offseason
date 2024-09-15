package org.firstinspires.ftc.opmodes;

import static org.firstinspires.ftc.objects.outtake.Claw.clawState;
import static org.firstinspires.ftc.objects.outtake.Claw.ndReleaseState;
import static org.firstinspires.ftc.objects.outtake.Claw.stReleaseState;
import static org.firstinspires.ftc.objects.outtake.Virtual4Bar.v4bState;
import static java.lang.Thread.sleep;

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

@Autonomous
@Config
public class Old_blue extends OpMode {

    public enum AutoInstruction {

        WAITING,

        INSTRUCTION_1, INSTRUCTION_1_1,
        INSTRUCTION_2,
        INSTRUCTION_3,
        INSTRUCTION_4,
        INSTRUCTION_5, INSTRUCTION_5_1,
        INSTRUCTION_6,
        INSTRUCTION_7,
        INSTRUCTION_8,
        INSTRUCTION_9,
        INSTRUCTION_10,
        INSTRUCTION_11,
        INSTRUCTION_12,
        INSTRUCTION_13,
        INSTRUCTION_14,
        INSTRUCTION_15,
        INSTRUCTION_16,
        INSTRUCTION_17,
        INSTRUCTION_18,
        INSTRUCTION_19,
        INSTRUCTION_20,
        INSTRUCTION_21,
        INSTRUCTION_22,
        INSTRUCTION_23,
        INSTRUCTION_24;

    }
    public static AutoInstruction autoInstruction = AutoInstruction.INSTRUCTION_1;

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

    public static double x = 0, y = 0, theta = 0;

    public static double x1, y1, h1 = 0;
    public static double x1_case3 = 75, y1_case3 = -40;
    public static double x1_case2 = 80, y1_case2 = -25;
    public static double x1_case1 = 75, y1_case1 = -15;

    public static double x1_1, y1_1, h1_1;
    public static double x1_1_case3 = 0, y1_1_case1 = 0, h1_1_case3 = 120;
    public static double x1_1_case2 = 0, y1_1_case2 = 0, h1_1_case2 = 90;
    public static double x1_1_case1 = 0, y1_1_case3 = 0, h1_1_case1 = 100;

    public static double x2, y2, h2 = 90;
    public static double x2_case3 = 35, y2_case3 = 28;
    public static double x2_case2 = 35, y2_case2 = 18;
    public static double x2_case1 = 45, y2_case1 = 0;

    public static double x3 = 0, y3 = 130, h3 = 90;

    public static double x4 = -82, y4 = 87, h4 = 90;
    public static double x4_case1 = -42, x4_case2 = -62, x4_case3 = -82;

    public static double x5 = -x4, y5 = -y4, h5 = 90;

    public static double x6 = 0, y6 = 110, h6 = 90;
    public static double x7 = -30, y7 = y4, h7 = 90;
    public static double x8 = -x7, y8 = -y4, h8 = 90;

    Extendo.ExtendoState extendoPickUpState;

    private double clx, cly, cltheta;

    private double translationError = 0;
    private double headingError = 0;

    public static double translationTolerance = 5.5;
    public static double headingTolerance = 3;

    private boolean firstCallInstruction_1 = true;
    private boolean firstCallInstruction_5_1 = true;
    private boolean firstCallInstruction_6 = true;
    private boolean firstCallInstruction_7 = true;
    private boolean firstCallInstruction_10 = true;
    private boolean firstCallInstruction_12 = true;
    private boolean firstCallInstruction_15 = true;
    private boolean firstCallInstruction_18 = true;
    private boolean firstCallInstruction_20 = true;
    private boolean firstCallInstruction_23 = true;



    private AutoInstruction nextInstruction;
    private double currentTimeout = 0;

    public static int zona = 3;

    @Override
    public void init() {

        RobotHardware.pipelineColour = PipelineStanga.team.albastru;


        autoInstruction = AutoInstruction.INSTRUCTION_1;

        StaticVariables.init(hardwareMap, telemetry, gamepad1, gamepad2);

        robot = new RobotHardware();
        robot.init();

        driveSubsystem = new DriveSubsystem(robot);
        odometry = new Odometry(robot);

        linearPath = new LinearPath(x1, y1, h1);

        v4b = new Virtual4Bar(robot);
        claw = new Claw(robot);
        lift = new Lift(robot);

        activeIntake = new ActiveIntake(robot);
        extendo = new Extendo(robot);
        commands = new Commands();

        Claw.stReleaseState = Claw.StReleaseState.CLOSED;

        extendo.reachedPositionDown = false;
        extendo.reachedPositionAutoRedCase3 = false;






    }

    @Override
    public void init_loop() {
        zona = PipelineStanga.region_of_interest;

        switch (zona) {

            case 1:

                x1 = x1_case1;
                y1 = y1_case1;

                x1_1 = x1_1_case1;
                y1_1 = y1_1_case1;
                h1_1 = h1_1_case1;

                x2 = x2_case1;
                y2 = y2_case1;

                x4 = x4_case1;

                extendoPickUpState = Extendo.ExtendoState.PICKUP_AUTO_RED_CASE3;


                break;

            case 2:

                x1 = x1_case2;
                y1 = y1_case2;

                x1_1 = x1_1_case2;
                y1_1 = y1_1_case2;
                h1_1 = h1_1_case2;

                x2 = x2_case2;
                y2 = y2_case2;

                x4 = x4_case2;

                extendoPickUpState = Extendo.ExtendoState.PICKUP_AUTO_RED_CASE2;


                break;

            case 3:

                x1 = x1_case3;
                y1 = y1_case3;

                x1_1 = x1_1_case3;
                y1_1 = y1_1_case3;
                h1_1 = h1_1_case3;

                x2 = x2_case3;
                y2 = y2_case3;

                x4 = x4_case3;

                extendoPickUpState = Extendo.ExtendoState.PICKUP_AUTO_RED_CASE1;


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

    private void calculateErrors () {

        translationError = Math.sqrt(Math.pow(x- StaticVariables.robotX, 2) + Math.pow(y-StaticVariables.robotY, 2));
        headingError = theta - Math.toDegrees(StaticVariables.robotTheta);
        if (headingError > 180) headingError -= 360;
        else if (headingError < -180) headingError += 360;

    }

    private void changePathTo (double x, double y, double h, boolean close) {
        StaticVariables.robotX = StaticVariables.robotX - this.x;
        StaticVariables.robotY = StaticVariables.robotY - this.y;

        /*if(close) {
            LinearPath.K = LinearPath.closeK;
            LinearPath.Kt = LinearPath.closeKt;
            LinearPath.Kn = LinearPath.closeKn;

            LinearPath.P = LinearPath.Pclose;
            LinearPath.D = LinearPath.Dclose;
        }
        else {
            LinearPath.K = LinearPath.farK;
            LinearPath.Kt = LinearPath.farKt;
            LinearPath.Kn = LinearPath.farKn;

            LinearPath.P = LinearPath.Pfar;
            LinearPath.D = LinearPath.Dfar;

        }*/

        this.x = x; this.y = y; this.theta = h;

        calculateErrors();

    }
    public int framebun = 0; //  TODO TUNEAZA PID
    private boolean reachedDest () {
        if(Math.abs(headingError) < headingTolerance) framebun++;
        else framebun = 0;
        if(Math.abs(translationError) < translationTolerance && Math.abs(headingError) < headingTolerance && framebun >= 30 )
            return true;
        return false;
    }

    private boolean firstCall = true;
    private void esperas (double seconds, AutoInstruction nextInstruction) {

        if(firstCall) {
            timer.reset();
            autoInstruction = AutoInstruction.WAITING;
            firstCall = false;
            this.nextInstruction = nextInstruction;
            currentTimeout = seconds;
        }
        if(!firstCall && timer.seconds() > seconds) {
            autoInstruction = this.nextInstruction;
            firstCall = true;
        }
    }

    @Override
    public void loop() {

        //zona = PipelineStanga.region_of_interest;




        if (x != clx || y != cly || theta != cltheta)
            linearPath = new LinearPath(x, y, theta);
        clx = x; cly = y; cltheta = theta;

        calculateErrors();

        switch (autoInstruction) {

            case WAITING:

                esperas(currentTimeout, nextInstruction);

                break;

            case INSTRUCTION_1:

                if(firstCallInstruction_1 == true) {

                    x = 0;
                    y = 0;
                    theta = 0;
                    extendo.reachedPositionDown = false;
                    extendo.reachedPositionAutoRedCase1 = false;
                    extendo.reachedPositionAutoRedCase2 = false;
                    extendo.reachedPositionAutoRedCase3 = false;

                    changePathTo(x1, y1, h1, false);
                    firstCallInstruction_1 = false;

                    ndReleaseState = Claw.NdReleaseState.TRANSFER;
                    Virtual4Bar.v4bState = Virtual4Bar.V4BState.RELEASE;
                    Claw.clawState = Claw.ClawState.RELEASE_AUTO;
                }



                if(reachedDest()) {
                    changePathTo(x1_1, y1_1, h1_1, true);
                    esperas(0.1, AutoInstruction.INSTRUCTION_1_1);

                }
                break;

            case INSTRUCTION_1_1:

                if(reachedDest()) {
                    autoInstruction = AutoInstruction.INSTRUCTION_2;
                }

                break;

            case INSTRUCTION_2:

                Claw.stReleaseState = Claw.StReleaseState.TRANSFER;
                Extendo.extendoState = extendoPickUpState;
                ActiveIntake.servoState =  ActiveIntake.ServoState.PIXEL_5;

                if(Extendo.reachedPositionAutoRedCase3 || Extendo.reachedPositionAutoRedCase2 || Extendo.reachedPositionAutoRedCase1) {

                    autoInstruction = AutoInstruction.INSTRUCTION_3;

                }

                break;

            case INSTRUCTION_3:

                Virtual4Bar.v4bState = Virtual4Bar.V4BState.INIT;
                Claw.clawState = Claw.ClawState.INIT;
                Extendo.extendoState = Extendo.ExtendoState.DOWN;
                ActiveIntake.servoState = ActiveIntake.ServoState.INTAKE;

                if(Extendo.reachedPositionDown) {
                    changePathTo(x2, y2, h2, true);
                    autoInstruction = AutoInstruction.INSTRUCTION_4;
                }


                break;

            case INSTRUCTION_4:

                if(reachedDest()) {
                    changePathTo(x3, y3, h3, false);
                    autoInstruction = AutoInstruction.INSTRUCTION_5;
                    ActiveIntake.servoState = ActiveIntake.ServoState.OUTTAKE;
                }

                break;

            case INSTRUCTION_5:

                if(reachedDest()) {

                    ActiveIntake.servoState = ActiveIntake.ServoState.RAISE;
                    Transfer.initiateTransfer = true;

                    esperas(1.5, AutoInstruction.INSTRUCTION_5_1);

                }

                break;

            case INSTRUCTION_5_1:

                if(firstCallInstruction_5_1) {
                    changePathTo(x4, y4, h4, false);
                    firstCallInstruction_5_1 = false;
                }

                lift.liftState = Lift.LiftState.AUTO_1;
                Virtual4Bar.v4bState = Virtual4Bar.V4BState.RELEASE;
                Claw.clawState = Claw.ClawState.RELEASE;

                if(reachedDest()) {

                    stReleaseState = Claw.StReleaseState.RELEASE_MID;
                    lift.liftState = Lift.LiftState.AUTO_2;

                    esperas(0.5, AutoInstruction.INSTRUCTION_6);

                }

                break;

            case INSTRUCTION_6:

                if(firstCallInstruction_6) {
                    ndReleaseState = Claw.NdReleaseState.RELEASE_OPEN;
                    esperas (0.5, AutoInstruction.INSTRUCTION_7);
                    firstCallInstruction_6 = false;
                }

                break;

            case INSTRUCTION_7:

                if(firstCallInstruction_7) {

                    changePathTo(x5, y5, h5, false);

                    v4bState = Virtual4Bar.V4BState.INIT;
                    clawState = Claw.ClawState.INIT;
                    stReleaseState = Claw.StReleaseState.INIT;
                    ndReleaseState = Claw.NdReleaseState.INIT;
                    lift.liftState = Lift.LiftState.GO_DOWN;

                    firstCallInstruction_7 = false;

                    autoInstruction = AutoInstruction.INSTRUCTION_8;

                }

                break;

            case INSTRUCTION_8:

                if(reachedDest()) {

                    changePathTo(x6, y6, h6, false);
                    Extendo.extendoState = Extendo.ExtendoState.UP;
                    ActiveIntake.servoState = ActiveIntake.ServoState.PIXEL_5;


                    esperas(2, AutoInstruction.INSTRUCTION_9);

                }

                break;

            case INSTRUCTION_9:

                activeIntake.servoState = ActiveIntake.ServoState.PIXEL_4;
                esperas(0.5, AutoInstruction.INSTRUCTION_10);


                break;

            case INSTRUCTION_10:

                if(firstCallInstruction_10) {

                    changePathTo(x6, -y6, h6, false);

                    Extendo.extendoState = Extendo.ExtendoState.DOWN;
                    activeIntake.servoState = ActiveIntake.ServoState.INTAKE;
                    esperas(0.2, AutoInstruction.INSTRUCTION_11);

                    firstCallInstruction_10 = false;

                }


                break;

            case INSTRUCTION_11:

                activeIntake.servoState = ActiveIntake.ServoState.OUTTAKE;
                if(Extendo.reachedPositionDown) {

                    activeIntake.servoState = ActiveIntake.ServoState.RAISE;
                    Transfer.initiateTransfer = true;
                    esperas(0.5, AutoInstruction.INSTRUCTION_12);

                }

                break;

            case INSTRUCTION_12:

                if(firstCallInstruction_12) {

                    changePathTo(x7, y7, h7, true);

                    lift.liftState = Lift.LiftState.AUTO_2;
                    Virtual4Bar.v4bState = Virtual4Bar.V4BState.RELEASE;
                    Claw.clawState = Claw.ClawState.RELEASE;

                    autoInstruction = AutoInstruction.INSTRUCTION_13;

                }

                break;

            case INSTRUCTION_13:

                if(reachedDest()) {

                    stReleaseState = Claw.StReleaseState.RELEASE_MID;
                    esperas(0.2, AutoInstruction.INSTRUCTION_14);

                }

                break;

            case INSTRUCTION_14:

                stReleaseState = Claw.StReleaseState.RELEASE_MID_OPEN;
                ndReleaseState = Claw.NdReleaseState.RELEASE_OPEN;

                esperas(0.2, AutoInstruction.INSTRUCTION_15);

                break;

            case INSTRUCTION_15:

                if(firstCallInstruction_15) {

                    changePathTo(x8, y8, h8, false);

                    v4bState = Virtual4Bar.V4BState.INIT;
                    clawState = Claw.ClawState.INIT;
                    stReleaseState = Claw.StReleaseState.INIT;
                    ndReleaseState = Claw.NdReleaseState.INIT;
                    lift.liftState = Lift.LiftState.GO_DOWN;

                    firstCallInstruction_15 = false;

                    autoInstruction = AutoInstruction.INSTRUCTION_16;

                }

                break;

            case INSTRUCTION_16:

                if(reachedDest()) {

                    changePathTo(x6, y6, h6, false);
                    Extendo.extendoState = Extendo.ExtendoState.UP;
                    ActiveIntake.servoState = ActiveIntake.ServoState.PIXEL_3;


                    esperas(2, AutoInstruction.INSTRUCTION_17);

                }

                break;

            case INSTRUCTION_17:

                activeIntake.servoState = ActiveIntake.ServoState.PIXEL_2;
                esperas(0.5, AutoInstruction.INSTRUCTION_18);

                break;

            case INSTRUCTION_18:

                if(firstCallInstruction_18) {

                    changePathTo(x6, -y6, h6, false);

                    Extendo.extendoState = Extendo.ExtendoState.DOWN;
                    activeIntake.servoState = ActiveIntake.ServoState.INTAKE;
                    esperas(0.2, AutoInstruction.INSTRUCTION_19);

                    firstCallInstruction_18 = false;

                }

                break;

            case INSTRUCTION_19:

                activeIntake.servoState = ActiveIntake.ServoState.OUTTAKE;
                if(Extendo.reachedPositionDown) {

                    activeIntake.servoState = ActiveIntake.ServoState.RAISE;
                    Transfer.initiateTransfer = true;
                    esperas(0.5, AutoInstruction.INSTRUCTION_20);

                }

                break;

            case INSTRUCTION_20:

                if(firstCallInstruction_20) {

                    changePathTo(x7, y7, h7, true);

                    lift.liftState = Lift.LiftState.AUTO_2;
                    Virtual4Bar.v4bState = Virtual4Bar.V4BState.RELEASE;
                    Claw.clawState = Claw.ClawState.RELEASE;

                    firstCallInstruction_20 = false;

                    autoInstruction = AutoInstruction.INSTRUCTION_21;

                }

                break;

            case INSTRUCTION_21:

                if(reachedDest()) {

                    stReleaseState = Claw.StReleaseState.RELEASE_MID;
                    esperas(0.2, AutoInstruction.INSTRUCTION_22);

                }

                break;

            case INSTRUCTION_22:

                stReleaseState = Claw.StReleaseState.RELEASE_MID_OPEN;
                ndReleaseState = Claw.NdReleaseState.RELEASE_OPEN;

                esperas(0.2, AutoInstruction.INSTRUCTION_23);

                break;


            case INSTRUCTION_23:

                if(firstCallInstruction_23) {


                    v4bState = Virtual4Bar.V4BState.INIT;
                    clawState = Claw.ClawState.INIT;
                    stReleaseState = Claw.StReleaseState.INIT;
                    ndReleaseState = Claw.NdReleaseState.INIT;
                    lift.liftState = Lift.LiftState.GO_DOWN;

                    firstCallInstruction_23 = false;

                    telemetry.addData("fin", "fin");

                }


                break;
        }









        linearPath.update();
        odometry.update();
        driveSubsystem.updateAuto(linearPath.getOutput_velocity_x(), linearPath.getOutput_velocity_y(), linearPath.getOutput_velocity_theta());

        updateObjects();

        commands.update();

        robot.update();

        telemetry.addData("error theta: ", headingError);
        telemetry.addData("error t: ", translationError);

        telemetry.update();


    }
}
