package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by mathnerd on 9/28/16.
 */

public class Team_10363_TeleOp_Decs {
    //motors
    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_right_drive;
    //CR Servos
    private CRServo v_servo_sweep;
    //sensors
    private GyroSensor SensorGyro;

    public void init(HardwareMap ahwMap) {
        /*Try to add the left drive motor, stop it if it's moving, set zero power behavior
        * to "brake", and turn encoders off. */
        try{
            v_motor_left_drive=ahwMap.dcMotor.get("left_drive");
            v_motor_left_drive.setDirection(DcMotor.Direction.FORWARD);
            v_motor_left_drive.setPower(0);
            v_motor_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //If it doesn't work, set the motor to null and add record the problem in the Debug log.
        catch (Exception p_exception){
            v_motor_left_drive=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        //Same as above for the right motor, but reversed
        try {
            v_motor_right_drive=ahwMap.dcMotor.get("right_drive");
            v_motor_right_drive.setDirection(DcMotor.Direction.REVERSE);
            v_motor_right_drive.setPower(0);
            v_motor_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        catch (Exception p_exception){
            v_motor_right_drive=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        //Try to add the gyro
        try {
            SensorGyro=ahwMap.gyroSensor.get("gyro");
        }
        catch (Exception p_exception){
            DbgLog.msg(p_exception.getLocalizedMessage());
            SensorGyro=null;
        }
        //Calibrate the gyro. Drive team- don't hit start until the light starts blinking!
        if (SensorGyro!=null){
            SensorGyro.calibrate();
            while(SensorGyro.isCalibrating()){
                try {
                    Thread.sleep(50);
                }
                catch (Exception p_exception){}
            }
        }
        try {
            v_servo_sweep=ahwMap.crservo.get("sweep");
            v_servo_sweep.setPower(0);
        }
        catch (Exception p_exception){
            DbgLog.msg(p_exception.getLocalizedMessage());
            v_servo_sweep=null;
        }


    }
    //Modifies the left drive motor's power
    public void m_left_drive_power(float power){
        if (v_motor_left_drive!=null){
            float sendpower= Range.clip(power,-1,1);
            v_motor_left_drive.setPower(sendpower);
        }

    }
    //Same as above, but for the right drive motor
    public void m_right_drive_power(float power){
        if (v_motor_right_drive!=null){
            float sendpower=Range.clip(power,-1,1);
            v_motor_right_drive.setPower(sendpower);
        }
    }
    //Sets drive motors' power. Same as in Auto.
    public void setDrivePower(float left_power, float right_power){
        m_left_drive_power(left_power);
        m_right_drive_power(right_power);

    }
    //Same as above, but sets both to average if both are close.
    public void setDrivePowerWithCorrection(float left_power, float right_power){
        if (Math.abs(left_power-right_power)<=.05){
            setDrivePower((left_power+right_power)/2,(right_power+left_power)/2);
        }
        else {
            setDrivePower(left_power,right_power);
        }
    }
    /* Does the calculations for single stick driving for the left drive motor.
    *  To test the algorithm, use single_stick_drive_test*/

    public double single_stick_drive_left(double joystick_x, double joystick_y){

        int quadrant;
        double theta=0;
        double thetaAdjusted;
        double xAdjustedForAngle;
        double yAdjustedForAngle;
        double xAdjustedForAngleAnd1_1;
        double r;
        double scale;
        //find out what quadrant you're in
        if (joystick_x>=0&&joystick_y>=0){quadrant=1;}
        else if (joystick_x<=0&&joystick_y>=0){quadrant=2;}
        else if (joystick_x<=0&&joystick_y<=0){quadrant=3;}
        else {quadrant=4;}
        if (joystick_x==0&&joystick_y>=0){theta=90;}
        else if (joystick_x==0){theta=270;}
        else if (quadrant==1) {
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x));
        }
        else if (quadrant==2){

            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+180;
        }
        else if (quadrant==3){
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+180;
        }
        else if (quadrant==4){
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+360;
        }
        //find r
        r=Math.sqrt((joystick_x*joystick_x)+(joystick_y*joystick_y));
        //find adjusted theta and x and y coordinates
        thetaAdjusted=theta-45;
        xAdjustedForAngle=r*(Math.cos(Math.toRadians(thetaAdjusted)));
        yAdjustedForAngle=r*(Math.sin(Math.toRadians(thetaAdjusted)));
        //scale power to max of (1,1)
        if (xAdjustedForAngle==0 && yAdjustedForAngle==0){scale=0;}
        else if (Math.abs(xAdjustedForAngle)>Math.abs(yAdjustedForAngle)){scale=Math.abs(1/Math.cos(Math.toRadians(thetaAdjusted)));}
        else {scale=Math.abs(1/Math.sin(Math.toRadians(thetaAdjusted)));}
        xAdjustedForAngleAnd1_1=xAdjustedForAngle*scale;
        return xAdjustedForAngleAnd1_1;

    }
    //Same as above, but for the right drive motor.
    public double single_stick_drive_right(double joystick_x, double joystick_y){

        int quadrant;
        double theta=0;
        double thetaAdjusted;
        double xAdjustedForAngle;
        double yAdjustedForAngle;
        double yAdjustedForAngleAnd1_1;
        double r;
        double scale;
        //find out what quadrant you're in
        if (joystick_x>=0&&joystick_y>=0){quadrant=1;}
        else if (joystick_x<=0&&joystick_y>=0){quadrant=2;}
        else if (joystick_x<=0&&joystick_y<=0){quadrant=3;}
        else {quadrant=4;}
        if (joystick_x==0&&joystick_y>=0){theta=90;}
        else if (joystick_x==0){theta=270;}
        else if (quadrant==1) {
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x));
        }
        else if (quadrant==2){

            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+180;
        }
        else if (quadrant==3){
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+180;
        }
        else if (quadrant==4){
            theta=Math.toDegrees(Math.atan(joystick_y/joystick_x))+360;
        }
        //find r
        r=Math.sqrt((joystick_x*joystick_x)+(joystick_y*joystick_y));
        //find adjusted theta and x and y coordinates
        thetaAdjusted=theta-45;
        xAdjustedForAngle=r*(Math.cos(Math.toRadians(thetaAdjusted)));
        yAdjustedForAngle=r*(Math.sin(Math.toRadians(thetaAdjusted)));
        //scale power to max of (1,1)
        if (xAdjustedForAngle==0 && yAdjustedForAngle==0){scale=0;}
        else if (Math.abs(xAdjustedForAngle)>Math.abs(yAdjustedForAngle)){scale=Math.abs(1/Math.cos(Math.toRadians(thetaAdjusted)));}
        else {scale=Math.abs(1/Math.sin(Math.toRadians(thetaAdjusted)));}
        yAdjustedForAngleAnd1_1=yAdjustedForAngle*scale;
        return yAdjustedForAngleAnd1_1;

    }
    /* accesses the gyro's heading. If we can't find it, return an incorrect value that is divisible
    *  by 360. Remember that the gyro returns a clockwise heading. */
    public int a_gyro_heading(){
        if (SensorGyro!=null){
            return SensorGyro.getHeading();
        }
        else {return -36000;}
    }
    //calculates adjspeed, the speed correction factor in straightDrive.
    public double adjspeed(double speedModifier, int deltaAngle){
        return speedModifier*Math.sin(Math.toRadians(deltaAngle));
    }
    //modifies sweep servo's speed
    public  void m_sweep_speed(double speed){
        if (v_servo_sweep!=null){
            v_servo_sweep.setPower(Range.clip(speed, -1, 1));
        }
    }
}