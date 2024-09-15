package org.firstinspires.ftc.commands;

import static org.firstinspires.ftc.objects.outtake.Claw.clawState;
import static org.firstinspires.ftc.objects.outtake.Claw.ndReleaseState;
import static org.firstinspires.ftc.objects.outtake.Claw.stReleaseState;
import static org.firstinspires.ftc.objects.outtake.Lift.liftState;
import static org.firstinspires.ftc.objects.outtake.Virtual4Bar.v4bState;
import static org.firstinspires.ftc.robot.StaticVariables.gamepad1;
import static org.firstinspires.ftc.robot.StaticVariables.lastgamepad1;
import static org.firstinspires.ftc.robot.StaticVariables.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.objects.outtake.Claw;
import org.firstinspires.ftc.objects.outtake.Lift;
import org.firstinspires.ftc.objects.outtake.Virtual4Bar;
public class Release {

    public enum ReleaseState {
        OFF,
        READY,
        STPIXEL,
        NDPIXEL,
        WAITING,
        FINISH;
    }

    public static ReleaseState releaseState, nextState;

    public static boolean initiateRelease;
    private ElapsedTime timer = new ElapsedTime();

    public Release() {

        initiateRelease = false;
        releaseState = ReleaseState.OFF;
        nextState = ReleaseState.OFF;

        timer.reset();
    }

    public void update() {

        if (initiateRelease) {

            releaseState = ReleaseState.READY;
            initiateRelease = false;
        }

        if (releaseState == ReleaseState.WAITING && timer.seconds() > 0.3) {
            releaseState = nextState;
        }

        switch (releaseState) {
            case READY:
                v4bState = Virtual4Bar.V4BState.RELEASE;
                clawState = Claw.ClawState.RELEASE;
                ndReleaseState = Claw.NdReleaseState.RELEASE_CLOSED;

                releaseState = ReleaseState.WAITING;
                nextState = ReleaseState.STPIXEL;
                timer.reset();
                break;

            case STPIXEL:
                if (gamepad1.right_stick_button && !lastgamepad1.right_stick_button) {
                    stReleaseState = Claw.StReleaseState.RELEASE_MID;

                    releaseState = ReleaseState.WAITING;
                    nextState = ReleaseState.NDPIXEL;
                    timer.reset();
                }
                break;

            case NDPIXEL:
                if (gamepad1.right_stick_button && !lastgamepad1.right_stick_button) {
                    //stReleaseState = Claw.StReleaseState.RELEASE_MID_OPEN;
                    clawState = Claw.ClawState.MAX_RANGE;
                    ndReleaseState = Claw.NdReleaseState.RELEASE_OPEN;

                    releaseState = ReleaseState.WAITING;
                    nextState = ReleaseState.FINISH;
                    timer.reset();
                }
                break;

            case FINISH:
                if (liftState == Lift.LiftState.MOVING_DOWN) {
                    v4bState = Virtual4Bar.V4BState.INIT;
                    clawState = Claw.ClawState.INIT;
                    stReleaseState = Claw.StReleaseState.INIT;
                    ndReleaseState = Claw.NdReleaseState.INIT;

                    releaseState = ReleaseState.OFF;

                }
                break;
        }



        telemetry.addData("Rlease State", releaseState);
        telemetry.addLine();
    }
}
