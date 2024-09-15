package org.firstinspires.ftc.commands;

import static org.firstinspires.ftc.commands.Release.initiateRelease;
import static org.firstinspires.ftc.commands.Release.releaseState;
import static org.firstinspires.ftc.commands.Transfer.initiateTransfer;
import static org.firstinspires.ftc.objects.intake.ActiveIntake.changeIntake;
import static org.firstinspires.ftc.objects.intake.ActiveIntake.servoState;
import static org.firstinspires.ftc.objects.intake.Extendo.extendoState;
import static org.firstinspires.ftc.robot.StaticVariables.lastgamepad1;
import static org.firstinspires.ftc.robot.StaticVariables.gamepad1;

import android.media.Image;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.objects.intake.ActiveIntake;
import org.firstinspires.ftc.objects.intake.Extendo;
import org.firstinspires.ftc.objects.PlaneLauncher;
import org.firstinspires.ftc.objects.outtake.Lift;

public class Commands {
    private Transfer transfer;
    private Release release;

    private boolean pressed, doubleClick;

    private ElapsedTime timer = new ElapsedTime();
    public Commands () {

        transfer = new Transfer();
        release = new Release();

        pressed = false;
        doubleClick = false;
    }

    public void update() {
        if (gamepad1.square == true && lastgamepad1.square == false) {
            initiateTransfer = true;
        }

        if (pressed) {
            if (doubleClick) {
                if (extendoState == Extendo.ExtendoState.DOWN) {
                    extendoState = Extendo.ExtendoState.UP;
                    servoState = ActiveIntake.ServoState.INTAKE;
                }
                else {
                    servoState = ActiveIntake.ServoState.RAISE;
                    initiateTransfer = true;
                }

                pressed = false;
                doubleClick = false;
            }
            else if (timer.seconds() > 0.25) {
                changeIntake = true;

                pressed = false;
            }

        }


        if (gamepad1.right_stick_button && !lastgamepad1.right_stick_button && releaseState == Release.ReleaseState.OFF) {
            if (!pressed) {
                pressed = true;
                timer.reset();
            }
            else
            {
                doubleClick = true;
            }
        }

        transfer.update();
        release.update();
    }
}
