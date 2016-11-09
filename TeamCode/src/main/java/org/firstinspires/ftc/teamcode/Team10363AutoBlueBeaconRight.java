package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Lego5 on 10/23/2016.
 */
@Autonomous(name="10363 Competition Autonomous on Blue Alliance (right beacon)")
public class Team10363AutoBlueBeaconRight extends OpMode{

    Team_10363_Auto_Decs RobII= new Team_10363_Auto_Decs();
    @Override
    public void init() {
        RobII.init(hardwareMap);
        left_encoder = RobII.a_left_encoder_pos();
        right_encoder = RobII.a_right_encoder_pos();
        telemetry.addData("4: Heading", RobII.a_gyro_heading());
        telemetry.addData("6: Ground Color (Alpha)", RobII.a_ground_alpha());
        telemetry.update();
    }

    @Override
    public void loop() {
//        RobII.m_intake_power(-1);
        telemetry.update();
        telemetry.addData("1: State",v_state);
        telemetry.addData("2: left encoder", RobII.a_left_encoder_pos());
        telemetry.addData("3: right encoder", RobII.a_right_encoder_pos());
        telemetry.addData("4: Heading", RobII.a_gyro_heading());
        telemetry.addData("5: Ground Color (Blue)", RobII.a_ground_blue());
        telemetry.addData("6: Ground Color (Alpha)", RobII.a_ground_alpha());
        telemetry.addData("7: Beacon Red", RobII.a_left_red());
        telemetry.addData("8: Beacon Blue", RobII.a_left_blue());
        telemetry.addData("9: last state left", left_encoder);
        telemetry.addData("10: last state right", right_encoder);
        switch (v_state) {
            case 0:
                RobII.setDrivePower((float) (.5 - RobII.adjspeed(1, RobII.a_gyro_heading())), (float) (.5 + RobII.adjspeed(1, RobII.a_gyro_heading())));
                if (RobII.have_drive_encoders_reached(left_encoder-4220, right_encoder-4220, false)){
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            case 1:
                RobII.setDrivePower((float) .4, 0);
                if (RobII.a_gyro_heading() >= 40 && RobII.a_gyro_heading() < 180) {
                    RobII.setDrivePower(0, 0);
                    v_state++;
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                }
                break;
            case 2:
                RobII.setDrivePower((float) (.3 - RobII.adjspeed(1, RobII.a_gyro_heading()-45)), (float) (.3 + RobII.adjspeed(1, RobII.a_gyro_heading()-45)));
                if (RobII.have_drive_encoders_reached(left_encoder-10524,right_encoder-10524,false)){
                    RobII.setDrivePower(0,0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            case 3:
                RobII.setDrivePower((float) -.3, (float) .3);
                if ((RobII.a_gyro_heading() <= 340) && (RobII.a_gyro_heading() > 290)){
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            case 4:
                if (RobII.a_ground_alpha()==255) {v_state=255; break;}  // if the color sensor isn't working, just stop autonomous
                RobII.setDrivePower((float) -0.2, (float) -0.2);
//                RobII.setDrivePower((float) (-.2 + RobII.adjspeed(2, RobII.a_gyro_heading()-340)), (float) (-.2 - RobII.adjspeed(2, RobII.a_gyro_heading()-340)));
                if (RobII.a_ground_alpha()>=7){
                    RobII.setDrivePower(0, 0);
                    left_encoder = RobII.a_left_encoder_pos();
                    right_encoder = RobII.a_right_encoder_pos();
                    v_state++;
                }
                if (RobII.have_drive_encoders_reached(left_encoder+4000,right_encoder+4000,true)){
                    RobII.setDrivePower(0, 0);
                    v_state++;
                }
                break;
            case 5:
                //Robert's Jukes. Also called LineFollower because it follows the white line based on gyro and color sensors.
                if (RobII.a_ground_alpha()>=7){
                    RobII.setDrivePower((float) -.2, (float) -.2);
                    telemetry.addData("11: Move State: ","sees white line");
                    telemetry.update();
                }
                else if (RobII.a_gyro_heading()>270){
                    RobII.setDrivePower((float) -.2, (float) .2);
                    telemetry.addData("11: Move State: ","gyro >270");
                    telemetry.update();
                }
                else if (RobII.a_gyro_heading()<270){
                    RobII.setDrivePower(.2f,-.2f);
                    telemetry.addData("11: Move State: ","gyro <270");
                    telemetry.update();
                }
                else {
                    RobII.setDrivePower(-.2f,-.2f);
                    telemetry.addData("11: Move State: ","gyro=270");
                    telemetry.update();
                }
                // Check for motors stalling
                if (leftEncoderProblems == RobII.a_left_encoder_pos() && rightEncoderProblems == RobII.a_right_encoder_pos()){
                    count = count + 1;
                }else {
                    count = 0;
                    leftEncoderProblems = RobII.a_left_encoder_pos();
                    rightEncoderProblems = RobII.a_right_encoder_pos();
                }
                // Is counter greater than 126 iters (3*42)? If so, skip to next state.
                if (count > 126){
                    RobII.setDrivePower(0.0f, 0.0f);
                    left_encoder=RobII.a_left_encoder_pos();
                    right_encoder=RobII.a_right_encoder_pos();
                    v_state++;
                }
                else if (RobII.a_left_encoder_pos()-left_encoder+RobII.a_right_encoder_pos()-right_encoder<=-7763){
                    RobII.setDrivePower(0.0f, 0.0f);
                    left_encoder=RobII.a_left_encoder_pos();
                    right_encoder=RobII.a_right_encoder_pos();
                    v_state++;
                }
                break;
            case 6:
                RobII.move_left_beacon_to_read();
                if (RobII.a_left_blue()>=2 && RobII.a_left_red()<2){
                    RobII.push_left_beacon();
                    v_state++;
                }
                else if (RobII.a_left_red()>=2 && RobII.a_left_blue()<2){
                    RobII.push_right_beacon();
                    RobII.reset_left_beacon_servo();
                    v_state++;
                }
                else {
                    RobII.reset_left_beacon_servo();
                    v_state++;
                }
                break;
            /*case 7:
                RobII.reset_left_beacon_servo();
                RobII.reset_right_beacon_servo();
                RobII.setDrivePower(-0.5f,-0.5f);
                if (RobII.have_drive_encoders_reached(left_encoder+4657,right_encoder+4657,true)){
                    RobII.setDrivePower(0,0);
                    left_encoder=RobII.a_left_encoder_pos();
                    right_encoder=RobII.a_right_encoder_pos();
                    v_state++;
                }

           */ default:
                telemetry.addData("200: ","Done!");
                break;
        }
    }
    int v_state=0;
    int left_encoder=0;
    int right_encoder=0;
    int count=0;
    int leftEncoderProblems=0;
    int rightEncoderProblems=0;



}
