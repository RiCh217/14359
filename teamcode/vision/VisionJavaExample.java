package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "VisionAuton")
public class VisionJavaExample extends LinearOpMode{
    private DcMotor frontleftDrive;
    private DcMotor frontrightDrive;
    private DcMotor backrightDrive;
    private DcMotor backleftDrive;
    private DcMotor lift;
    private ElapsedTime T1 = new ElapsedTime();
    MasterVision vision;
    SampleRandomizedPositions goldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        frontleftDrive = hardwareMap.get(DcMotor.class, "fl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "fr");
        backleftDrive = hardwareMap.get(DcMotor.class, "bl");
        backrightDrive = hardwareMap.get(DcMotor.class, "br");
        lift = hardwareMap.get(DcMotor.class, "lift");

        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);

        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "AeKPEzP/////AAABmUqowS+DiEKrnjNVXcnhreQhzTt//uW/QLlTnbwQVTtnNY4uxJOJ+w/asMLSp1KhiFYDdJU7MBzfoZyKxeEn2d+8YF+gEf+lExVNZt9f1CSwVUxkMvzzDhdPC5uKsHvvW40tYAF0Lx10n/0KFrirId+7m50t6p6ZF0eO9eKyt7rMqNJDsf+9kdU2cZVO3BxQmuUZqn4QdV4HngWjIuCHAB/ctXWZX8fDU58q+4EDyh/W16sBxMCezpPj9v0xi4Qio5govn7TgBu6nvUz2DJi9EbrwDGgZE33DNOONQDOVYj/Dl7oMsv/IRSOf0gzkKcPUWkijnhVRuAvlZFkUjom/g+e3wffIgDiZ437jBNesfOz";

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_NONE);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time

        waitForStart();

        vision.disable();// disables tracking algorithms. this will free up your phone's processing power for other jobs.

        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        while(opModeIsActive()){
            telemetry.addData("goldPosition was", goldPosition);// giving feedback

            switch (goldPosition){ // using for things in the autonomous program
                case LEFT:
                    telemetry.addLine("going to the left");
                    break;
                case CENTER:
                    telemetry.addLine("going straight");
                    break;
                case RIGHT:
                    telemetry.addLine("going to the right");

                    break;
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    break;
            }

            telemetry.update();
        }

        vision.shutdown();
    }


    public void drive(double power){
        backleftDrive.setPower(power);
        backrightDrive.setPower(power);
        frontleftDrive.setPower(power);
        frontrightDrive.setPower(power);
    }

    public void turn(double power){ //positive clockwise
        backleftDrive.setPower(-power);
        backrightDrive.setPower(power);
        frontleftDrive.setPower(-power);
        frontrightDrive.setPower(power);
    }

    int degreesToTicks(double degree){
        double ticksPerTurn = 1000; //not sure MUST CHANGE
        double ticks = degree * ticksPerTurn/360;
        return (int)ticks;
    }

    int fixTimerValue(int rawSeconds){
        int milliseconds;
        milliseconds = rawSeconds * 1000;
        if(milliseconds<250){
            milliseconds = 250;
        }
        return milliseconds;
    }

    public void rightTurn(double power){ //positive clockwise
        backrightDrive.setPower(power);
        frontrightDrive.setPower(power);
    }

    public void leftTurn(double power){ //positive clockwise
        backleftDrive.setPower(-power);
        frontleftDrive.setPower(-power);
    }

    public void pidDrive(int target, double waitTime, double maxPower){

        double kP = 0;
        double kI = 0;
        double kD = 0;

        int error;
        double proportion;

        double integralRaw = 0;
        double integral;
        double integralPowerLimit = .6/kI;

        double lastError = 0;
        double derivative;

        double integralActiveZone = 1/8 * target;

        double finalPower;
        boolean timerBool = true;
        T1.reset();
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(T1.time()<fixTimerValue((int)waitTime)){

            error = target - (backrightDrive.getCurrentPosition() + backleftDrive.getCurrentPosition());
            proportion = kP * error;

            if(Math.abs(error) < integralActiveZone && error != 0){
                integralActiveZone =+ error;
            }
            else{
                integralRaw = 0;
            }
            if(integralRaw >integralPowerLimit){
                integralRaw = integralPowerLimit;
            }
            if(integralRaw < -integralPowerLimit){
                integralRaw = -integralPowerLimit;
            }


            integral = kI * integralRaw;

            derivative = kD * (error - lastError);
            lastError = error;

            if(error == 0){
                derivative = 0;
            }


            finalPower = proportion + integral + derivative;

            if(finalPower>maxPower*1){
                finalPower = maxPower*1;
            }
            else if(finalPower<-maxPower*1){
                finalPower = -maxPower*1;
            }

            drive(finalPower);

            sleep(100);

            if(error<30){
                timerBool = false;
            }

            if(timerBool){
                T1.reset();
            }
        }
        drive(0);

    }

    public void pidTurn(double degrees, double waitTime, double maxPower){

        double kP = 0;
        double kI = 0;
        double kD = 0;
        double kP_C = 0;

        int error;
        double proportion;

        double integralRaw = 0;
        double integral;
        double integralPowerLimit = .6/kI;
        double integralActiveZone = 1/8 * degrees;


        double lastError = 0;
        double derivative;

        double error_drift;
        double proportion_drift;

        double finalPower;
        boolean timerBool = true;
        T1.reset();
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(T1.time()<fixTimerValue((int)waitTime)){

            error = degreesToTicks(degrees) - (backrightDrive.getCurrentPosition() - backleftDrive.getCurrentPosition());
            proportion = kP * error;

            if(Math.abs(error) < integralActiveZone && error != 0){
                integralActiveZone =+ error;
            }
            else{
                integralRaw = 0;
            }
            if(integralRaw >integralPowerLimit){
                integralRaw = integralPowerLimit;
            }
            if(integralRaw < -integralPowerLimit){
                integralRaw = -integralPowerLimit;
            }


            integral = kI * integralRaw;

            derivative = kD * (error - lastError);
            lastError = error;

            if(error == 0){
                derivative = 0;
            }


            finalPower = proportion + integral + derivative;

            if(finalPower>maxPower*1){
                finalPower = maxPower*1;
            }
            else if(finalPower<-maxPower*1){
                finalPower = -maxPower*1;
            }

            error_drift = backrightDrive.getCurrentPosition()-backleftDrive.getCurrentPosition();
            proportion_drift = kP_C * error_drift;


            rightTurn(finalPower + proportion_drift);
            leftTurn(finalPower - proportion_drift);

            sleep(100);

            if(error<30){
                timerBool = false;
            }

            if(timerBool){
                T1.reset();
            }
        }
        turn(0);

    }
}
