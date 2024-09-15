package org.firstinspires.ftc.objects.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robot.RobotHardware;

@Config
public class Extendo {

    public static boolean reachedPositionAutoRedCase3 = false;
    public static boolean reachedPositionAutoRedCase2 = false;
    public static boolean reachedPositionAutoRedCase1 = false;
    public static boolean reachedPositionDown = false;
    public static boolean reachedPositionUp = false;

    private RobotHardware robot;
    private DcMotor motorExtendo;
    public enum ExtendoState {
        DOWN,
        UP,
        READY_FOR_TRANSFER,
        TRANSFER,

        PICKUP_AUTO_RED_CASE3,
        PICKUP_AUTO_RED_CASE2,
        PICKUP_AUTO_RED_CASE1,

        AUTO_FINISH;
    }
    public static ExtendoState extendoState, lastExtendoState;

    private final int CPR = 145 * 4, RPM = 1150;
    private final int VELOCITY = CPR * RPM / 60; // counts / second   (max power)

    private final int LOWER_BOUND = 15;
    private final int UPPER_BOUND = 1680;
    public static int TRANSFER_POSITION = 55;

    private final int PICKUP_AUTO_RED_CASE3 = 1100;
    private final int PICKUP_AUTO_RED_CASE2 = 700;
    private final int PICKUP_AUTO_RED_CASE1 = 200;

    private final int AUTO_FINISH = -5;

    private int framecounter = 0;

    public Extendo (RobotHardware robot) {
        this.robot = robot;

        motorExtendo = robot.motorExtendo;

        extendoState = ExtendoState.DOWN;
    }

    public void update() {
        motorExtendo.setPower(1);

        if(lastExtendoState != extendoState) {

            switch (extendoState) {
                case DOWN:

                    if (!reachedPositionDown)
                        motorExtendo.setTargetPosition(LOWER_BOUND);

                    if (Math.abs(motorExtendo.getCurrentPosition() - LOWER_BOUND) < 5)
                        reachedPositionDown = true;
                    else reachedPositionDown = false;


                    break;

                case UP:

                    if (!reachedPositionUp)
                        motorExtendo.setTargetPosition(UPPER_BOUND);

                    if (Math.abs(motorExtendo.getCurrentPosition() - UPPER_BOUND) < 5)
                        reachedPositionUp = true;
                    else reachedPositionUp = false;

                    break;

                case READY_FOR_TRANSFER:
                    motorExtendo.setTargetPosition(TRANSFER_POSITION);

                    if (Math.abs(motorExtendo.getCurrentPosition() - TRANSFER_POSITION) < 5)
                        extendoState = ExtendoState.TRANSFER;

                    break;

                case TRANSFER:
                    motorExtendo.setTargetPosition(TRANSFER_POSITION);

                    break;

                case PICKUP_AUTO_RED_CASE3:

                    if (!reachedPositionAutoRedCase3)
                        motorExtendo.setTargetPosition(PICKUP_AUTO_RED_CASE3);

                    if (Math.abs(motorExtendo.getCurrentPosition() - PICKUP_AUTO_RED_CASE3) < 5)
                        reachedPositionAutoRedCase3 = true;
                    else reachedPositionAutoRedCase3 = false;


                    break;

                case PICKUP_AUTO_RED_CASE2:

                    if (!reachedPositionAutoRedCase2)
                        motorExtendo.setTargetPosition(PICKUP_AUTO_RED_CASE2);

                    if (Math.abs(motorExtendo.getCurrentPosition() - PICKUP_AUTO_RED_CASE2) < 5)
                        reachedPositionAutoRedCase2 = true;
                    else reachedPositionAutoRedCase2 = false;


                    break;

                case PICKUP_AUTO_RED_CASE1:

                    if (!reachedPositionAutoRedCase1)
                        motorExtendo.setTargetPosition(PICKUP_AUTO_RED_CASE1);

                    if (Math.abs(motorExtendo.getCurrentPosition() - PICKUP_AUTO_RED_CASE1) < 5)
                        reachedPositionAutoRedCase1 = true;
                    else reachedPositionAutoRedCase1 = false;


                    break;

                case AUTO_FINISH:

                    motorExtendo.setTargetPosition(AUTO_FINISH);

                    break;
            }
        }

    }
}
