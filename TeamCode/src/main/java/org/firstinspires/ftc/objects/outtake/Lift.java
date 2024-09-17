package org.firstinspires.ftc.objects.outtake;

import static org.firstinspires.ftc.commands.Release.initiateRelease;
import static org.firstinspires.ftc.commands.Release.releaseState;
import static org.firstinspires.ftc.robot.StaticVariables.gamepad1;
import static org.firstinspires.ftc.robot.StaticVariables.lastgamepad1;
import static org.firstinspires.ftc.robot.StaticVariables.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.commands.Release;
import org.firstinspires.ftc.robot.RobotHardware;

public class Lift {
    private RobotHardware robot;
    private DcMotor motorLiftLeft, motorLiftRight;

    public enum LiftState {
        DOWN,
        MOVING_MID,
        MOVING_UP,
        MID,
        UP,
        MOVING_DOWN,

        AUTO_1,
        AUTO_2,
        GO_DOWN,

        AUTO_FINISH;
    }

    public static LiftState liftState, lastLiftState;

    public static int kLift = 80;

    private ElapsedTime timer = new ElapsedTime();
    private boolean ok;

    private final int LOWER_BOUND = 0;
    private final int MIDDLE_BOUND = 1300;
    private final int UPPER_BOUND = 2000;
    private final int AUTO_FINISH = 0;

    private final int HANG_UP = 1550;
    private final int HANG_DOWN = 400;

    private final int AUTO_1 = 700;
    private final int AUTO_2 = 800;

    private final int tolerance = 10;

    private boolean hangToggle = false;

    public Lift(RobotHardware robot) {
        this.robot = robot;

        motorLiftLeft = robot.motorLiftLeft;
        motorLiftRight = robot.motorLiftRight;

        liftState = LiftState.DOWN;
        ok = false;
    }

    public void update() {

        telemetry.addData("motor position: ", motorLiftLeft.getCurrentPosition());

        motorLiftLeft.setPower(1);
        motorLiftRight.setPower(1);

        if(gamepad1.right_trigger > 0.1) {

            motorLiftLeft.setTargetPosition((int) (motorLiftLeft.getCurrentPosition() + gamepad1.right_trigger * kLift));
            motorLiftRight.setTargetPosition((int) (motorLiftRight.getCurrentPosition() + gamepad1.right_trigger * kLift));

        }

        if(gamepad1.left_trigger > 0.1) {

            motorLiftLeft.setTargetPosition((int) (motorLiftLeft.getCurrentPosition() - gamepad1.left_trigger * kLift));
            motorLiftRight.setTargetPosition((int) (motorLiftRight.getCurrentPosition() - gamepad1.left_trigger * kLift));

        }

        if (liftState == LiftState.MOVING_MID || liftState == LiftState.MOVING_UP && timer.seconds() > 0.1 && ok) {
            initiateRelease = true;
            ok = false;
        }

        if(gamepad1.dpad_right == true && lastgamepad1.dpad_right == false) {

            if(hangToggle == true) {
                motorLiftLeft.setTargetPosition(HANG_DOWN);
                motorLiftRight.setTargetPosition(HANG_DOWN);
                Virtual4Bar.v4bState = Virtual4Bar.V4BState.HANG;
                hangToggle = !hangToggle;
            }
            else {
                motorLiftLeft.setTargetPosition(HANG_UP);
                motorLiftRight.setTargetPosition(HANG_UP);
                Virtual4Bar.v4bState = Virtual4Bar.V4BState.HANG;
                hangToggle = !hangToggle;
            }

        }

        if(lastLiftState != liftState) {

            switch (liftState) {
                case DOWN:
                    if (gamepad1.left_stick_button == true && lastgamepad1.left_stick_button == false) {
                        motorLiftLeft.setTargetPosition(MIDDLE_BOUND);
                        motorLiftRight.setTargetPosition(MIDDLE_BOUND);
                        liftState = LiftState.MOVING_MID;
                        timer.reset();
                        ok = true;

                    } else if (gamepad1.left_bumper == true && lastgamepad1.left_bumper == false) {
                        motorLiftLeft.setTargetPosition(UPPER_BOUND);
                        motorLiftRight.setTargetPosition(UPPER_BOUND);
                        liftState = LiftState.MOVING_UP;
                        timer.reset();
                        ok = true;
                    }
                    break;
                case MOVING_MID:
                    if (Math.abs(motorLiftLeft.getCurrentPosition() - MIDDLE_BOUND) < tolerance) {
                        liftState = LiftState.UP;
                    }
                    break;
                case MOVING_UP:
                    if (Math.abs(motorLiftLeft.getCurrentPosition() - UPPER_BOUND) < tolerance) {
                        liftState = LiftState.UP;
                    }
                    break;

                case MID:
                    motorLiftLeft.setTargetPosition(MIDDLE_BOUND);
                    motorLiftRight.setTargetPosition(MIDDLE_BOUND);
                    break;

                case UP:
                    if (gamepad1.right_bumper == true && lastgamepad1.right_bumper == false) {
                        motorLiftLeft.setTargetPosition(LOWER_BOUND);
                        motorLiftRight.setTargetPosition(LOWER_BOUND);
                        liftState = LiftState.MOVING_DOWN;

                        releaseState = Release.ReleaseState.FINISH;
                    }

                    break;
                case MOVING_DOWN:
                    if (Math.abs(motorLiftLeft.getCurrentPosition() - LOWER_BOUND) < tolerance) {
                        liftState = LiftState.DOWN;
                    }
                    break;

                case AUTO_1:

                    motorLiftRight.setTargetPosition(AUTO_1);
                    motorLiftLeft.setTargetPosition(AUTO_1);

                    break;

                case AUTO_2:

                    motorLiftRight.setTargetPosition(AUTO_2);
                    motorLiftLeft.setTargetPosition(AUTO_2);

                    break;

                case GO_DOWN:

                    motorLiftRight.setTargetPosition(LOWER_BOUND);
                    motorLiftLeft.setTargetPosition(LOWER_BOUND);

                    break;

                case AUTO_FINISH:

                    motorLiftLeft.setTargetPosition(AUTO_FINISH);
                    motorLiftRight.setTargetPosition(AUTO_FINISH);

                    break;


            }
        }

    }

}
