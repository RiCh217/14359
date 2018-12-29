package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auton")
public class Auton extends LinearOpMode {
    private DcMotor frontleftDrive;
    private DcMotor frontrightDrive;
    private DcMotor backrightDrive;
    private DcMotor backleftDrive;
    private CRServo rollerServo;
    private CRServo armServo;
    private CRServo slideServo;
    private DcMotor arm;
    private DcMotor lift;
    private ColorSensor colorSensor;


    @Override
    public void runOpMode() {

        frontleftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");
        backleftDrive = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");
        rollerServo = hardwareMap.get(CRServo.class, "roller");
        armServo = hardwareMap.get(CRServo.class, "armS");
        slideServo = hardwareMap.get(CRServo.class, "slide");
        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");
        colorSensor = hardwareMap.colorSensor.get("color");

        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);

        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        slideOut(2500);
        armSDown(1000);
        slideIn(1750);
        armSUp(1500);
        turnPower(.55);
        sleep(1000);
        stopDrive();
        driveTime(.3, 150);
        sleep(600);



    }


    private void drivePower(double power){

        backleftDrive.setPower(power);
        backrightDrive.setPower(power);
        frontleftDrive.setPower(power);
        frontrightDrive.setPower(power);
    }
    private void driveTime(double power, int time){

        backleftDrive.setPower(power);
        backrightDrive.setPower(power);
        frontleftDrive.setPower(power);
        frontrightDrive.setPower(power);
        sleep(time);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
    }
    private void turnPower(double power){

        backleftDrive.setPower(-power);
        backrightDrive.setPower(-power);
        frontleftDrive.setPower(power);
        frontrightDrive.setPower(power);
    }

    private void stopDrive(){
        drivePower(0);
    }

/*
    private void drive(double power, int ticks){

        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleftDrive.setTargetPosition(ticks);
        backrightDrive.setTargetPosition(ticks);

        drivePower(power);

        while(backrightDrive.isBusy() && backleftDrive.isBusy()){

        }

        stopDrive();
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    */

    private void turn(double power, int ticks){

        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleftDrive.setTargetPosition(-ticks);
        backrightDrive.setTargetPosition(ticks);

        turnPower(power);

        while(backrightDrive.isBusy() && backleftDrive.isBusy()){

        }

        stopDrive();
    }

    private void outtake(int time){
        rollerServo.setPower(1);
        sleep(time);
        rollerServo.setPower(0);
    }

    private void slideOut(int time){
        slideServo.setPower(-1);
        sleep(time);
        slideServo.setPower(0);
    }

    private void slideIn(int time){
        slideServo.setPower(1);
        sleep(time);
        slideServo.setPower(0);
    }

    private void armSDown(int time){
        armServo.setPower(1);
        sleep(time);
        armServo.setPower(0);
    }

    private void armSUp(int time){
        armServo.setPower(-1);
        sleep(time);
        armServo.setPower(0);
    }

    private void liftUp(double power, int time){
        lift.setPower(-power);
        sleep(time);
        lift.setPower(0);
    }

    private void liftDown(double power, int time){
        lift.setPower(power);
        sleep(time);
        lift.setPower(0);
    }



}



