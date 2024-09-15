package org.firstinspires.ftc.objects.intake;

import static org.firstinspires.ftc.robot.StaticVariables.gamepad1;
import static org.firstinspires.ftc.robot.StaticVariables.lastgamepad1;
import static org.firstinspires.ftc.robot.StaticVariables.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot.RobotHardware;
public class ActiveIntake {
    private enum MotorState {
        FORWARD,
        REVERSE,
        OFF;
    }

    public enum ServoState {
        INTAKE,
        PIXEL_2,
        PIXEL_3,
        PIXEL_4,
        PIXEL_5,
        OUTTAKE,
        RAISE;
    }

    private MotorState motorState, lastMotorState;
    public static ServoState servoState, lastServoState;
    private RobotHardware robot;
    private DcMotor motorIntake;
    private Servo servoIntake;

    private final double INTAKE_POSITION = 0.65;
    private final double PIXEL_2_POSITION = 0.46; // DE CALCULAT
    private final double PIXEL_3_POSITION = 0.48; // DE CALCULAT
    private final double PIXEL_4_POSITION = 0.5; // DE CALCULAT
    private final double PIXEL_5_POSITION = 0.52; // DE CALCULAT
    private final double OUTTAKE_POSITION = 0.13; // DE CALCULAT
    private final double RAISE_POSITION = 0.45;
    private final double power = 0.6;
    public static double position = 0;


    private ElapsedTime timer = new ElapsedTime();
    private boolean pressed;
    public static boolean changeIntake;

    public ActiveIntake(RobotHardware robot) {
        this.robot = robot;

        motorIntake = robot.motorIntake;
        servoIntake = robot.servoIntake;

        motorState = MotorState.OFF;
        servoState = ServoState.RAISE;

        servoIntake.setPosition(RAISE_POSITION);

        pressed = false;
        changeIntake = false;
    }

    public void update() {

        if (changeIntake) {
            if (servoState == ServoState.RAISE) servoState = ServoState.INTAKE;
            else servoState = ServoState.RAISE;

            changeIntake = false;
        }

        if(gamepad1.circle) {

            servoState = ServoState.OUTTAKE;

        }


        if(lastServoState != servoState) {

            switch (servoState) {
                case INTAKE:
                    servoIntake.setPosition(INTAKE_POSITION);
                    motorState = MotorState.FORWARD;
                    break;
                case RAISE:
                    servoIntake.setPosition(RAISE_POSITION);
                    motorState = MotorState.OFF;
                    break;

                case PIXEL_2:
                    servoIntake.setPosition(PIXEL_2_POSITION);
                    motorState = MotorState.FORWARD;
                    break;

                case PIXEL_3:
                    servoIntake.setPosition(PIXEL_3_POSITION);
                    motorState = MotorState.FORWARD;
                    break;

                case PIXEL_4:
                    servoIntake.setPosition(PIXEL_4_POSITION);
                    motorState = MotorState.FORWARD;
                    break;

                case PIXEL_5:
                    servoIntake.setPosition(PIXEL_5_POSITION);
                    motorState = MotorState.FORWARD;
                    break;

                case OUTTAKE:
                    servoIntake.setPosition(OUTTAKE_POSITION);
                    motorState = MotorState.REVERSE;
                    break;
            }
        }

        if(lastMotorState != motorState) {

            switch (motorState) {
                case OFF:
                    motorIntake.setPower(0);
                    break;
                case FORWARD:
                    motorIntake.setPower(power);
                    break;
                case REVERSE:
                    motorIntake.setPower(-power);
                    break;
            }
        }

    }
}
