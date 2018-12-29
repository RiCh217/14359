package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name = "AdanDrive")
public class AdanOPMode extends LinearOpMode {
    private DcMotor frontleftDrive;
    private DcMotor frontrightDrive;
    private DcMotor backrightDrive;
    private DcMotor backleftDrive;
    private DcMotor intake;
    private DcMotor intakeArm;
    private DcMotor slide;
    private DcMotor arm;
    private DcMotor lift;

    @Override
    public void runOpMode() {
        frontleftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");
        backleftDrive = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeArm = hardwareMap.get(DcMotor.class, "intakeArm");
        slide= hardwareMap.get(DcMotor.class, "slide");
        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");



        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();


        while (opModeIsActive()) {

            //ArmServo
            if (gamepad1.dpad_up) {
                intakeArm.setPower(1);
            }
            else if (gamepad1.dpad_down) {
                intakeArm.setPower(-1);
            }
            else {
                intakeArm.setPower(0);
            }

            //SlideServo
            if (gamepad1.left_trigger>0) {
                slide.setPower(-1);
            }
            else if (gamepad1.left_bumper) {
                slide.setPower(1);
            }
            else {
                slide.setPower(0);
            }

            //RollerServo
            if (gamepad1.b) {
                intake.setPower(1);
            }
            else if (gamepad1.x) {
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }

            //Lift
            if (gamepad1.a) {
                lift.setPower(.6);
            }
            else if (gamepad1.y) {
                lift.setPower(-.6);
            }
            else {
                lift.setPower(0);
            }

            //Arm
            if (gamepad1.right_bumper) {
                arm.setPower(.5);
            }
            else if (gamepad1.right_trigger>0) {
                arm.setPower(-.5);
            }
            else {
                arm.setPower(0);
            }


            ////////DRIVE CODE
            //driving back and forward
            frontleftDrive.setPower(-gamepad1.left_stick_y);
            frontrightDrive.setPower(-gamepad1.left_stick_y);
            backleftDrive.setPower(-gamepad1.left_stick_y);
            backrightDrive.setPower(-gamepad1.left_stick_y);
            //driving left and right
            frontleftDrive.setPower(gamepad1.right_stick_x);
            backleftDrive.setPower(gamepad1.right_stick_x);
            backrightDrive.setPower(-gamepad1.right_stick_x);
            frontrightDrive.setPower(-gamepad1.right_stick_x);

        }
    }

}

