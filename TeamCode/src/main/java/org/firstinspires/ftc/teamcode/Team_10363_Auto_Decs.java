package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by Lego5 on 9/25/2016.
 */
public class Team_10363_Auto_Decs {
    //motors
    private DcMotor v_motor_left_drive;
    private DcMotor v_motor_right_drive;
    //servos
    private Servo v_servo_left_beacon;
    private Servo v_servo_right_beacon;
    //sensors
    private GyroSensor SensorGyro;
    private ColorSensor GroundColor;
    private ColorSensor LeftColor;
    private ColorSensor RightColor;

    public void init(HardwareMap ahwMap) {
        /*Try to add the left drive motor, stop it if it's moving, set zero power behavior
        * to "brake", and turn encoders on. */
        try{
            v_motor_left_drive=ahwMap.dcMotor.get("left_drive");
            v_motor_left_drive.setDirection(DcMotor.Direction.REVERSE);
            v_motor_left_drive.setPower(0);
            v_motor_left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        //If it doesn't work, set the motor to null and add record the problem in the Debug log.
        catch (Exception p_exception){
            v_motor_left_drive=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        //Same as above for the right motor, but reversed
        try {
            v_motor_right_drive=ahwMap.dcMotor.get("right_drive");
            v_motor_right_drive.setDirection(DcMotor.Direction.FORWARD);
            v_motor_right_drive.setPower(0);
            v_motor_right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            v_motor_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        catch (Exception p_exception){
            v_motor_right_drive=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        //Try to add the beacon pushing servos. The right one is in reverse.
        try{
            v_servo_left_beacon=ahwMap.servo.get("left beacon");
            v_servo_left_beacon.setDirection(Servo.Direction.FORWARD);
            v_servo_left_beacon.setPosition(.3);

        }
        catch (Exception p_exception){
            v_servo_left_beacon=null;
            DbgLog.msg(p_exception.getLocalizedMessage());
        }
        try{
            v_servo_right_beacon=ahwMap.servo.get("right beacon");
            v_servo_right_beacon.setDirection(Servo.Direction.REVERSE);
            v_servo_right_beacon.setPosition(.3);
        }
        catch (Exception p_exception){
            v_servo_right_beacon=null;
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
        //Try to add the color sensors
        try {
            GroundColor = ahwMap.colorSensor.get("ground");
            GroundColor.setI2cAddress(I2cAddr.create7bit(0x3C));
            GroundColor.enableLed(true);
        }
        catch (Exception p_exception){
            DbgLog.msg(p_exception.getLocalizedMessage());
            GroundColor=null;
        }
        try{
            LeftColor = ahwMap.colorSensor.get("left-color");
            LeftColor.setI2cAddress(I2cAddr.create7bit(0x42));
            LeftColor.enableLed(false);
        }
        catch (Exception p_exception){
            DbgLog.msg(p_exception.getLocalizedMessage());
            LeftColor=null;
        }
        try {
            RightColor = ahwMap.colorSensor.get("left-color");
            RightColor.setI2cAddress(I2cAddr.create7bit(0x44));
            RightColor.enableLed(false);
        }
        catch (Exception p_exception){
            DbgLog.msg(p_exception.getLocalizedMessage());
            RightColor=null;
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

    // Sets both drive motors at once. In teleOp, will have correction if close.
    public void setDrivePower(float left_power, float right_power){
        m_left_drive_power(left_power);
        m_right_drive_power(right_power);

    }
    //calculates adjspeed, the speed correction factor in straightDrive.
    public double adjspeed(double speedModifier, int deltaAngle){
        return speedModifier*Math.sin(Math.toRadians(deltaAngle));
    }
    /* accesses the gyro's heading. If we can't find it, return an incorrect value that is divisible
    *  by 360. Remember that the gyro returns a clockwise heading. */
    public int a_gyro_heading(){
        if (SensorGyro!=null){
            return SensorGyro.getHeading();
        }
        else {return -36000;}
    }
    //Resets the left drive wheel encoder.
    public void reset_left_drive_encoder ()

    {
        if (v_motor_left_drive != null)
        {
            v_motor_left_drive.setMode
                    ( DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    );
        }

    }
    //Resets the right drive wheel encoder.
    public void reset_right_drive_encoder ()

    {
        if (v_motor_right_drive != null)
        {
            v_motor_right_drive.setMode
                    ( DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    );
        }

    }
    //Resets both encoders
    public void reset_drive_encoders(){
        reset_left_drive_encoder();
        reset_right_drive_encoder();
    }
    //Gets the left drive motor's encoder pos
    public int a_left_encoder_pos(){
        if (v_motor_left_drive!=null){
            return v_motor_left_drive.getCurrentPosition();
        }
        else {
            return 0;
        }
    }
    //Same as above, but for the right drive motor
    public int a_right_encoder_pos(){
        if (v_motor_right_drive!=null){
            return v_motor_right_drive.getCurrentPosition();
        }
        else {
            return 0;
        }
    }
    //Have the drive encoders reached a certain value?
    public boolean have_drive_encoders_reached(int left, int right, boolean forwards){
        if (forwards){
            return a_left_encoder_pos()>=left&&a_right_encoder_pos()>=right;
        }
        else {
            return a_left_encoder_pos()<=left&&a_right_encoder_pos()<=right;
        }
    }
    public void push_left_beacon(){
        if (v_servo_left_beacon!=null){v_servo_left_beacon.setPosition(.6);}
    }
    public void move_left_beacon_to_read(){
        if (v_servo_left_beacon!=null){v_servo_left_beacon.setPosition(.45);}
    }
    public void reset_left_beacon_servo(){
        if (v_servo_left_beacon!=null) {v_servo_left_beacon.setPosition(.3);}
    }
    public void push_right_beacon(){
        if (v_servo_right_beacon!=null) {v_servo_right_beacon.setPosition(.6);}
    }
    public void reset_right_beacon_servo(){
        if (v_servo_right_beacon!=null) {v_servo_right_beacon.setPosition(.3);}
    }
    public double a_ground_blue(){
        if (GroundColor!=null) {
            return GroundColor.blue();
        }
        else return 0;
    }
    public double a_ground_alpha(){
        return GroundColor.alpha();

    }
    public double a_left_blue(){
        double returnthis = 0;
        if (LeftColor != null) {
            returnthis = LeftColor.blue();
        }
        return returnthis;
    }
    public double a_left_red(){
        double returnthis = 0;
        if (LeftColor != null) {
            returnthis = LeftColor.red();
        }
        return returnthis;
    }

}
