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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Disabled
@TeleOp
public class BACKUPROBOT extends LinearOpMode {
    public DcMotorEx arm = null;
    public DcMotorEx wrist = null;
    public Servo claw = null;
    public CRServo intake = null;
    public static final int ARM_POS_INIT = 300;
    public static final int ARM_POS_INTAKE = 450;
    public static final int ARM_POS_WALL_GRAB = 1100;
    public static final int ARM_POS_WALL_UNHOOK = 1700;
    public static final int ARM_POS_HOVER_HIGH = 2600;
    public static final int ARM_POS_CLIP_HIGH = 2100;
    public static final int ARM_POS_LOW_BASKET = 2500;

    public static final int WRIST_POS_INIT = 0;
    public static final int WRIST_POS_SAMPLE = 270;
    public static final int WRIST_POS_SPEC = 10;

    public static final double CLAW_OPEN_POS = -1;
    public static final double CLAW_CLOSED_POS = 1;

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

        private int targetArm = 0;
        private int targetWrist = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        arm = hardwareMap.get(DcMotorEx.class, "tower_Lift");
        wrist = hardwareMap.get(DcMotorEx.class, "hand");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        claw.scaleRange(-1,1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(13.8, -61.7, Math.toRadians(-90)));
            waitForStart();
            while (opModeIsActive()) {
                switch (currentState) {
                    case INIT:
                        targetArm = ARM_POS_INIT;
                        targetWrist = WRIST_POS_INIT;
                        telemetry.addData("State", "INIT");
                        break;
                    case INTAKE:
                        targetArm = ARM_POS_INTAKE;
                        targetWrist = WRIST_POS_SAMPLE;
                        telemetry.addData("State", "INTAKE");
                        break;
                    case WALL_GRAB:
                        targetArm = ARM_POS_WALL_GRAB;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "WALL_GRAB");
                        break;
                    case WALL_UNHOOK:
                        targetArm = ARM_POS_WALL_UNHOOK;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "WALL_UNHOOK");
                        break;

                    case HOVER_HIGH:
                        targetArm = ARM_POS_HOVER_HIGH;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "HOVER_HIGH");
                        break;

                    case CLIP_HIGH:
                        targetArm = ARM_POS_CLIP_HIGH;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "CLIP_HIGH");
                        break;
                    case LOW_BASKET:
                        targetArm = ARM_POS_LOW_BASKET;
                        targetWrist = WRIST_POS_SAMPLE;
                        telemetry.addData("State", "LOW_BASKET");
                        break;
                    case MANUAL:
                        telemetry.addData("State", "MANUAL");
                        break;
                }

                // Handle state transitions based on gamepad input
                if (gamepad2.a) {
                    currentState = RobotState.INTAKE;
                } else if (gamepad2.b && !lastGrab) {
                    if(currentState == RobotState.WALL_GRAB){
                        currentState = RobotState.WALL_UNHOOK;
                    }else{
                        currentState = RobotState.WALL_GRAB;
                    }
                } else if (gamepad2.y && !lastHook) {
                    if(currentState == RobotState.HOVER_HIGH){
                        currentState = RobotState.CLIP_HIGH;
                    }else{
                        currentState = RobotState.HOVER_HIGH;
                    }
                } else if (gamepad2.x) {
                    currentState = RobotState.LOW_BASKET;
                } else if (gamepad2.left_bumper) {
                    currentState = RobotState.INIT;
                } else if (gamepad2.dpad_up){ //manual control
                    currentState = RobotState.MANUAL;
                    targetArm += 10;
                } else if (gamepad2.dpad_down){
                    currentState = RobotState.MANUAL;
                    targetArm -= 10;
                } else if (gamepad2.dpad_left){
                    currentState = RobotState.MANUAL;
                    targetWrist += 1;
                } else if (gamepad2.dpad_right){
                    currentState = RobotState.MANUAL;
                    targetWrist -= 1;
                }

                lastGrab = gamepad2.b;
                lastHook = gamepad2.y;

                // Toggle claw position when right_bumper is pressed
                if (gamepad2.right_bumper && !lastBump) {
                    clawOpen = !clawOpen;
                    if (clawOpen) {
                        claw.setPosition(CLAW_OPEN_POS);
                    } else {
                        claw.setPosition(CLAW_CLOSED_POS);
                    }
                }
                lastBump = gamepad1.dpad_right;

                // Control intake servo with triggers
                if (gamepad2.right_trigger>0.1) {
                    intake.setPower(1.0);
                } else if (gamepad2.left_trigger>0.1) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0);
                }

                arm.setTargetPosition(targetArm);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setTargetPosition(targetWrist);
                wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(1);
                wrist.setPower(1);


                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
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

                drive.updatePoseEstimate();
                // Send telemetry data to the driver station
                telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
                telemetry.addData("Arm Position", arm.getCurrentPosition());
                telemetry.addData("Arm Power", arm.getPower());
                telemetry.addData("Wrist Position", wrist.getCurrentPosition());
                telemetry.addData("Wrist Power", wrist.getPower());
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
