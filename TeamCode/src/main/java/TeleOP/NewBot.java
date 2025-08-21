package TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.Arrays;
import java.util.Base64;
import java.util.List;

@Disabled
@TeleOp
public class NewBot extends LinearOpMode {
    public class Lift {
        private final DcMotorEx left_lift;
        private final DcMotorEx right_lift;

        public Lift(HardwareMap hardwareMap) {
            left_lift = hardwareMap.get(DcMotorEx.class, "left_lift");
            right_lift = hardwareMap.get(DcMotorEx.class, "right_lift");
            right_lift.setDirection(DcMotorSimple.Direction.REVERSE);
            left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            List<DcMotorEx> lift = Arrays.asList(left_lift, right_lift);
        }
    }
    public DcMotorEx arm = null;
    public DcMotorEx chain = null;
    public Servo wrist = null;
    public CRServo intake = null;
    public static final int LIFT_POS_INIT = -200;
    public static final int LIFT_POS_INTAKE = -350;
    public static final int LIFT_POS_WALL_GRAB = -1100;
    public static final int LIFT_POS_WALL_UNHOOK = -1700;
    public static final int LIFT_POS_HOVER_HIGH = -2600;
    public static final int LIFT_POS_CLIP_HIGH = -2100;
    public static final int LIFT_POS_LOW_BASKET = -2500;

    public static final int CHAIN_POS_INIT = 600;
    public static final int CHAIN_POS_SAMPLE = 0;
    public static final int CHAIN_POS_WALL_SPEC= 0;

    public static final int WRIST_POS_INIT = 0;
    public static final int WRIST_POS_SAMPLE = 1;
    public static final int WRIST_POS_SPEC = 2;

    public enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER_HIGH,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
        }
        private RobotState currentState = RobotState.INIT;

        public boolean clawOpen = true;
        public boolean lastBump = false;
        public boolean lastHook = false;
        public boolean lastGrab = false;

        private int targetLift = 0;
        private int targetWrist = 0;
        private int targetChain = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        chain = hardwareMap.get(DcMotorEx.class, "chain");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, Math.toRadians(-90)));
            Lift lift = new Lift(hardwareMap);
            waitForStart();
            while (opModeIsActive()) {
                /*switch (currentState) {
                    case INIT:
                        targetLift = LIFT_POS_INIT;
                        targetWrist = WRIST_POS_INIT;
                        targetChain = CHAIN_POS_INIT;
                        telemetry.addData("State", "INIT");
                        break;
                    case INTAKE:
                        targetLift = LIFT_POS_INTAKE;
                        targetWrist = WRIST_POS_SAMPLE;
                        telemetry.addData("State", "INTAKE");
                        break;
                    case WALL_GRAB:
                        targetLift = LIFT_POS_WALL_GRAB;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "WALL_GRAB");
                        break;
                    case WALL_UNHOOK:
                        targetLift = LIFT_POS_WALL_UNHOOK;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "WALL_UNHOOK");
                        break;

                    case HOVER_HIGH:
                        targetLift = LIFT_POS_HOVER_HIGH;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "HOVER_HIGH");
                        break;

                    case CLIP_HIGH:
                        targetLift = LIFT_POS_CLIP_HIGH;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "CLIP_HIGH");
                        break;
                    case LOW_BASKET:
                        targetLift = LIFT_POS_LOW_BASKET;
                        targetWrist = WRIST_POS_SAMPLE;
                        telemetry.addData("State", "LOW_BASKET");
                        break;
                    case MANUAL:
                        telemetry.addData("State", "MANUAL");
                        break;
                }

                // Handle state transitions based on gamepad input
                if (gamepad2.a) {
                    currentState = NewBot.RobotState.INTAKE;
                } else if (gamepad2.b && !lastGrab) {
                    if(currentState == NewBot.RobotState.WALL_GRAB){
                        currentState = NewBot.RobotState.WALL_UNHOOK;
                    }else{
                        currentState = NewBot.RobotState.WALL_GRAB;
                    }
                } else if (gamepad2.y && !lastHook) {
                    if(currentState == NewBot.RobotState.HOVER_HIGH){
                        currentState = NewBot.RobotState.CLIP_HIGH;
                    }else{
                        currentState = NewBot.RobotState.HOVER_HIGH;
                    }
                } else if (gamepad2.x) {
                    currentState = NewBot.RobotState.LOW_BASKET;
                } else if (gamepad2.left_bumper) {
                    currentState = NewBot.RobotState.INIT;
                } else if (gamepad2.dpad_up){ //manual control
                    currentState = NewBot.RobotState.MANUAL;
                    targetLift -= 10;
                } else if (gamepad2.dpad_down){
                    currentState = NewBot.RobotState.MANUAL;
                    targetLift += 10;
                } else if (gamepad2.dpad_left){
                    currentState = NewBot.RobotState.MANUAL;
                    targetWrist += 1;
                } else if (gamepad2.dpad_right){
                    currentState = NewBot.RobotState.MANUAL;
                    targetWrist -= 1;
                }*/

                if (gamepad1.a) {
                    lift.left_lift.setPower(1);
                } else lift.left_lift.setPower(0);
                lastGrab = gamepad2.b;
                lastHook = gamepad2.y;
                if(gamepad1.b) {
                    lift.right_lift.setPower(1);
                }else {
                    lift.right_lift.setPower(0);
                }


                // Handle state transitions based on gamepad input

                // Control intake servo with triggers
                if (gamepad2.right_trigger>0.1) {
                    intake.setPower(-1.0);
                } else if (gamepad2.left_trigger>0.1) {
                    intake.setPower(1.0);
                } else {
                    intake.setPower(0);
                }

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                gamepad1.left_stick_y,
                                gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x

                ));
                if (gamepad1.right_bumper) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y/4,
                                    -gamepad1.left_stick_x/4
                            ),
                            -gamepad1.right_stick_x/2
                    ));
                }

                //lift.left_lift.setTargetPosition(targetLift);
                //lift.right_lift.setTargetPosition(lift.left_lift.getTargetPosition());
                //lift.left_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //lift.right_lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(targetWrist);
                //lift.left_lift.setPower(0.7);
                //lift.right_lift.setPower(0.7);

                chain.setTargetPosition(targetChain);
                chain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                chain.setPower(0.7);

                //wrist.setPower(1);

                drive.updatePoseEstimate();
                // Send telemetry data to the driver station

                telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
                telemetry.addData("lift Position", lift.left_lift.getCurrentPosition());
                telemetry.addData("Lift Power", lift.left_lift.getPower());
                telemetry.addData("chain Position", chain.getCurrentPosition());
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();
            }
        } else {
            throw new RuntimeException();
        }
    }
}
