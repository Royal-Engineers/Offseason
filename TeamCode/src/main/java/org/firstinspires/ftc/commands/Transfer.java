package org.firstinspires.ftc.commands;

import static org.firstinspires.ftc.objects.intake.Extendo.extendoState;
import static org.firstinspires.ftc.objects.outtake.Claw.clawState;
import static org.firstinspires.ftc.objects.outtake.Claw.ndReleaseState;
import static org.firstinspires.ftc.objects.outtake.Claw.stReleaseState;
import static org.firstinspires.ftc.objects.outtake.Virtual4Bar.v4bState;
import static org.firstinspires.ftc.robot.StaticVariables.gamepad1;
import static org.firstinspires.ftc.robot.StaticVariables.lastgamepad1;
import static org.firstinspires.ftc.commands.Release.initiateRelease;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.objects.intake.Extendo;
import org.firstinspires.ftc.objects.outtake.Claw;
import org.firstinspires.ftc.objects.outtake.Virtual4Bar;
import org.firstinspires.ftc.robot.RobotHardware;
public class Transfer {

    private ElapsedTime timer = new ElapsedTime();

    public enum TransferState {
        OFF,
        MIDDLE_DOWN,
        DOWN,
        PICK_UP,
        THIRD_UP,
        MIDDLE_UP,
        UP,
        FINISH,
        WAITING;

    }
    public TransferState transferState, nextState;

    public static boolean initiateTransfer = false;

    public Transfer() {

        transferState = TransferState.OFF;
        nextState = TransferState.OFF;
    }

    public void update() {
        if (initiateTransfer == true) {
            transferState = TransferState.WAITING;
            nextState = TransferState.MIDDLE_DOWN;

            v4bState = Virtual4Bar.V4BState.INIT;
            clawState = Claw.ClawState.INIT;
            stReleaseState = Claw.StReleaseState.BASIC;
            ndReleaseState = Claw.NdReleaseState.INIT;
            extendoState = Extendo.ExtendoState.READY_FOR_TRANSFER;

            initiateTransfer = false;
            timer.reset();
        }

        if(transferState == TransferState.WAITING) {
            if (nextState == TransferState.MIDDLE_DOWN ) {
                if (extendoState == Extendo.ExtendoState.TRANSFER) transferState = nextState;
            }
            else if (timer.seconds() > 0.15) transferState = nextState;

        }

        switch (transferState) {
            case MIDDLE_DOWN:
                v4bState = Virtual4Bar.V4BState.TRANSFER_MID;
                clawState = Claw.ClawState.TRANSFER_MID;
                ndReleaseState = Claw.NdReleaseState.TRANSFER;

                transferState = TransferState.WAITING;
                nextState = TransferState.DOWN;
                timer.reset();
                break;

            case DOWN:
                v4bState = Virtual4Bar.V4BState.TRANSFER_DOWN;
                clawState = Claw.ClawState.TRANSFER_DOWN;
                stReleaseState = Claw.StReleaseState.TRANSFER;


                transferState = TransferState.WAITING;
                nextState = TransferState.PICK_UP;
                timer.reset();
                break;

            case PICK_UP:
                stReleaseState = Claw.StReleaseState.CLOSED;

                transferState = TransferState.WAITING;
                nextState = TransferState.THIRD_UP;
                timer.reset();
                break;

            case THIRD_UP:
                v4bState = Virtual4Bar.V4BState.TRANSFER_THIRD_UP;
                clawState = Claw.ClawState.TRANSFER_THIRD_UP;

                transferState = TransferState.WAITING;
                nextState = TransferState.MIDDLE_UP;
                timer.reset();
                break;

            case MIDDLE_UP:
                v4bState = Virtual4Bar.V4BState.TRANSFER_MID_UP;
                clawState = Claw.ClawState.TRANSFER_MID_UP;

                transferState = TransferState.WAITING;
                nextState = TransferState.UP;
                timer.reset();
                break;

            case UP:
                v4bState = Virtual4Bar.V4BState.INIT;
                clawState = Claw.ClawState.INIT;
                ndReleaseState = Claw.NdReleaseState.INIT;

                transferState = TransferState.WAITING;
                nextState = TransferState.FINISH;
                timer.reset();
                break;

            case FINISH:
                extendoState = Extendo.ExtendoState.DOWN;

                transferState = TransferState.OFF;

                break;
        }
    }
}
