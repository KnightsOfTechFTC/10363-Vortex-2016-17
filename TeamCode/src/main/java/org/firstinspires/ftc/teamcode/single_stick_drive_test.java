package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Lego5 on 9/11/2016.
 */
@TeleOp (name = "Single Stick Driving test")
public class single_stick_drive_test extends OpMode {

    @Override
    public void init() {
        // init motors
        try {
            v_motor_left_drive=hardwareMap.dcMotor.get("left_drive");
        }
        catch (Exception p_exception)
        {
           v_motor_left_drive=null;
        }
        try {
            v_motor_right_drive=hardwareMap.dcMotor.get("left_drive");
        }
        catch (Exception p_exception)
        {
            v_motor_right_drive=null;
        }

    }


    @Override
    public void loop() {
        x=gamepad1.left_stick_x;
        y=gamepad1.left_stick_y;
        telemetry.update();
        telemetry.addData("1: joystick x",x);
        telemetry.addData("2: joystick y",y);
        //find out what quadrant you're in
        if (x>=0&&y>=0){quadrant=1;}
        else if (x<=0&&y>=0){quadrant=2;}
        else if (x<=0&&y<=0){quadrant=3;}
        else {quadrant=4;}
        telemetry.update();
        telemetry.addData("3: quadrant", quadrant);
        //find theta
        if (x==0&&y>=0){theta=90;}
        else if (x==0){theta=270;}
        else if (quadrant==1) {
            theta=Math.toDegrees(Math.atan(y/x));
        }
        else if (quadrant==2){

            theta=Math.toDegrees(Math.atan(y/x))+180;
        }
        else if (quadrant==3){
            theta=Math.toDegrees(Math.atan(y/x))+180;
        }
        else if (quadrant==4){
            theta=Math.toDegrees(Math.atan(y/x))+360;
        }
        telemetry.update();
        telemetry.addData("4: theta",theta);
        //find r
        r=Math.sqrt((x*x)+(y*y));
        //find adjusted theta and x and y coordinates
        thetaAdjusted=theta-45;
        telemetry.update();
        telemetry.addData("5: adjusted theta",thetaAdjusted);
        xAdjustedForAngle=r*(Math.cos(thetaAdjusted));
        yAdjustedForAngle=r*(Math.sin(thetaAdjusted));
        telemetry.update();
        telemetry.addData("6: motor 1 input pre-adjustment for speed", xAdjustedForAngle);
        telemetry.addData("7: motor 2 input pre-adjustment for speed", yAdjustedForAngle);
        //x is an on/off button for scaling
        if (gamepad1.x && !xbuttonPress){
            xbuttonPress=true;
            AdjustSpeedTo1_1=!AdjustSpeedTo1_1;
        }
        else if (!gamepad1.x){xbuttonPress=false;}
        telemetry.update();
        telemetry.addData("8: speed adjustment to max of (1,1)",AdjustSpeedTo1_1);
        if (AdjustSpeedTo1_1){
            if (xAdjustedForAngle==0 && yAdjustedForAngle==0){scale=0;}
            else if (Math.abs(xAdjustedForAngle)>Math.abs(yAdjustedForAngle)){scale=Math.abs(1/Math.cos(Math.toRadians(thetaAdjusted)));}
            else {scale=Math.abs(1/Math.sin(Math.toRadians(thetaAdjusted)));}
            xAdjustedForAngleAnd1_1=xAdjustedForAngle*scale;
            yAdjustedForAngleAnd1_1=yAdjustedForAngle*scale;
        }
        else {
            xAdjustedForAngleAnd1_1=xAdjustedForAngle;
            yAdjustedForAngleAnd1_1=yAdjustedForAngle;
        }
        telemetry.update();
        telemetry.addData("9: motor 1 input with speed adjustment", xAdjustedForAngleAnd1_1);
        telemetry.addData("10: motor 2 input with speed adjustment", yAdjustedForAngleAnd1_1);
        //a is an on/off button for making motor inputs equal if they are close
        if (gamepad1.a && !abuttonPress){
            abuttonPress=true;
            allowmotorsequal=!allowmotorsequal;
        }
        else if (!gamepad1.a){abuttonPress=false;}
        if (allowmotorsequal){
            if (theta>80 && theta<100){motorsequal=true;}
            else if (theta>260 && theta<280){motorsequal=true;}
            else {motorsequal=false;}
            if (motorsequal){
                motor1Input=(xAdjustedForAngleAnd1_1+yAdjustedForAngleAnd1_1)/2;
                motor2input=(xAdjustedForAngleAnd1_1+yAdjustedForAngleAnd1_1)/2;
            }
            else {
                motor1Input=xAdjustedForAngleAnd1_1;
                motor2input=yAdjustedForAngleAnd1_1;
            }
        }
        else {
            motor1Input=xAdjustedForAngleAnd1_1;
            motor2input=yAdjustedForAngleAnd1_1;
        }
        telemetry.update();
        telemetry.addData("11: motor 1 final input", motor1Input);
        telemetry.addData("12: motor 2 final input", motor2input);
        v_motor_left_drive.setPower(motor1Input);
        v_motor_right_drive.setPower(motor2input);

    }
    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_right_drive;
    private double x;
    private double y;
    private int quadrant;
    private double theta;
    private double thetaAdjusted;
    private double xAdjustedForAngle;
    private double yAdjustedForAngle;
    private double xAdjustedForAngleAnd1_1;
    private double yAdjustedForAngleAnd1_1;
    private double r;
    private boolean AdjustSpeedTo1_1=true;
    private boolean xbuttonPress=false;
    private boolean abuttonPress=false;
    private double scale;
    private boolean allowmotorsequal=true;
    private boolean motorsequal=false;
    private double motor1Input;
    private double motor2input;

}
