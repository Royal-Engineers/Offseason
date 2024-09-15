package org.firstinspires.ftc.robot;

import static org.firstinspires.ftc.robot.StaticVariables.gamepad1;
import static org.firstinspires.ftc.robot.StaticVariables.gamepad2;
import static org.firstinspires.ftc.robot.StaticVariables.hardwareMap;
import static org.firstinspires.ftc.robot.StaticVariables.lastgamepad1;
import static org.firstinspires.ftc.robot.StaticVariables.lastgamepad2;
import static org.firstinspires.ftc.robot.StaticVariables.robotX;
import static org.firstinspires.ftc.robot.StaticVariables.robotY;
import static org.firstinspires.ftc.robot.StaticVariables.robotTheta;
import static org.firstinspires.ftc.robot.StaticVariables.telemetry;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.objects.chassis.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.objects.chassis.SwerveModule;
import org.firstinspires.ftc.pipelinuri.PipelineStanga;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class RobotHardware {
    private OpenCvPipeline pipeline;

    public OpenCvCamera camera;
    public static PipelineStanga.team pipelineColour;

    // HUBS
    public List<LynxModule> allHubs;
    public static int zona = 2;

    // ACTIVE INTAKE
    public DcMotor motorExtendo;
    public DcMotor motorIntake;
    public Servo servoIntake;

    // OUTTAKE
    public DcMotor motorLiftLeft, motorLiftRight; // BELLINGHAM
    public Servo servoV4BLeft, servoV4BRight;
    public Servo servoClaw, servoStRelease, servoNdRelease;
    public Servo servoAvion;


    // SWERVE
    public DcMotor motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft; // CHANGE TO DCMOTOR CLASS
    public CRServo servoFrontRight, servoFrontLeft, servoBackRight, servoBackLeft;
    public AnalogInput aencoderFrontRight, aencoderFrontLeft, aencoderBackRight, aencoderBackLeft;
    public AbsoluteAnalogEncoder encoderFrontRight, encoderFrontLeft, encoderBackLeft, encoderBackRight;
    public SwerveModule moduleFrontRight, moduleFrontLeft, moduleBackLeft, moduleBackRight;


    // OFFSETS
    public double FrontRight = 211;

    public double BackRight = 191;

    public double FrontLeft = 330;

    public double BackLeft = 215;

    public void init() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        pipeline = new PipelineStanga(telemetry,  pipelineColour);
        camera.setPipeline(pipeline);
        zona = PipelineStanga.region_of_interest;


        // HUBS
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // ACTIVE INTAKE
        motorExtendo = hardwareMap.get(DcMotor.class, "motorExtendo");

        motorExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorExtendo.setTargetPosition(15);
        motorExtendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorExtendo.setPower(1);

        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        servoIntake = hardwareMap.get(Servo.class, "servoIntake");

        // OUTTAKE
        motorLiftLeft = hardwareMap.get(DcMotor.class, "motorLiftLeft");
        motorLiftRight = hardwareMap.get(DcMotor.class, "motorLiftRight");

        motorLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftLeft.setTargetPosition(0);
        motorLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLiftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftRight.setTargetPosition(0);
        motorLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoV4BLeft = hardwareMap.get(Servo.class, "servoV4BLeft");
        servoV4BRight = hardwareMap.get(Servo.class, "servoV4BRight");

        servoClaw = hardwareMap.get(Servo.class, "servoClaw");
        servoStRelease = hardwareMap.get(Servo.class, "servoStRelease");
        servoNdRelease = hardwareMap.get(Servo.class, "servoNdRelease");

        servoAvion = hardwareMap.get(Servo.class, "servoAvion");

        // SWERVE
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoFrontRight = hardwareMap.get(CRServo.class, "servoFrontRight");
        servoFrontLeft = hardwareMap.get(CRServo.class, "servoFrontLeft");
        servoBackRight = hardwareMap.get(CRServo.class, "servoBackRight");
        servoBackLeft = hardwareMap.get(CRServo.class, "servoBackLeft");

        aencoderFrontRight = hardwareMap.get(AnalogInput.class, "encoderFrontRight");
        aencoderFrontLeft = hardwareMap.get(AnalogInput.class, "encoderFrontLeft");
        aencoderBackRight = hardwareMap.get(AnalogInput.class, "encoderBackRight");
        aencoderBackLeft = hardwareMap.get(AnalogInput.class, "encoderBackLeft");

        encoderFrontRight = new AbsoluteAnalogEncoder(aencoderFrontRight, FrontRight, true);
        encoderFrontLeft = new AbsoluteAnalogEncoder(aencoderFrontLeft, FrontLeft, true);
        encoderBackLeft = new AbsoluteAnalogEncoder(aencoderBackLeft, BackLeft, true);
        encoderBackRight = new AbsoluteAnalogEncoder(aencoderBackRight, BackRight, true);

        moduleFrontRight = new SwerveModule(motorFrontRight, servoFrontRight, encoderFrontRight);
        moduleFrontLeft = new SwerveModule(motorFrontLeft, servoFrontLeft, encoderFrontLeft);
        moduleBackLeft = new SwerveModule(motorBackLeft, servoBackLeft, encoderBackLeft);
        moduleBackRight = new SwerveModule(motorBackRight, servoBackRight, encoderBackRight);

        // ODOMETRY
        robotX = 0; robotY = 0; robotTheta = 0;

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    boolean cameraon = true;
    public void update() {
        if ( cameraon ) {
            camera.setPipeline(null);
            cameraon = false;
        }
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }


        lastgamepad1.copy(gamepad1);
        lastgamepad2.copy(gamepad2);
    }

}


// ip 192.168.43.1