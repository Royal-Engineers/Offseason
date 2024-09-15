package org.firstinspires.ftc.objects;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robot.StaticVariables.gamepad1;

import org.firstinspires.ftc.robot.RobotHardware;
@Config
public class PlaneLauncher {

    private RobotHardware robot;
    private Servo servoAvion;

    public static enum ServoState {
        CLOSE,
        OPEN;
    }

    public static ServoState servoState;
    public static ServoState lastServoState;

    public static double OPEN_POSITION = 0.3;
    public static double CLOSE_POSITION = 0.7;

    private void setPosition(double position) {
        servoAvion.setPosition(position);
    }

    public PlaneLauncher(RobotHardware robot) {
        this.robot = robot;

        servoAvion = robot.servoAvion;

        servoState = ServoState.CLOSE;
        setPosition(CLOSE_POSITION);

    }



    public void update() {

        if(gamepad1.dpad_left) {

            servoState = ServoState.OPEN;

        }


        if(lastServoState != servoState) {

            switch (servoState) {
                case CLOSE:
                    setPosition(CLOSE_POSITION);
                    lastServoState = servoState;
                    break;

                case OPEN:
                    setPosition(OPEN_POSITION);
                    lastServoState = servoState;
                    break;

            }

        }

    }

}
