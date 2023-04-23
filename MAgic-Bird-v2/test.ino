/*
 * @FilePath: \MAgIC-Bird\main\main.ino
 * @Date: 2023-03-20 19:13:54
 * @LastEditTime: 2023-03-20 20:23:05
 * @LastEditors: Xiaozhu Lin
 * @E-Mail: linxzh@shanghaitech.edu.cn
 * @Institution: MAgIC Lab, ShanghaiTech University, China
 */


#include <math.h>
#include <Ticker.h>
#include <Wire.h>
#include <PPMReader.h>


// PPM related params
byte ppm_pin = 0;
byte channel_num = 8;
PPMReader ppm(ppm_pin, channel_num);

// SERVO related params
#define SERVO_FREQ 50
#define SERVO_BIT 10
int servo_pin[2] = {1, 2};
int servo_chan[2] = {0, 1};
float servo_central_angle[2] = {90.0, 90.0};
float servo_angle[2] = {servo_central_angle[0], servo_central_angle[1]};

// MOTOR related params
#define MOTOR_FREQ 66
#define MOTOR_BIT 10
int motor_pin = 3;
int motor_chan = 2;
float motor_central_duty = 10;
float motor_duty = motor_central_duty;

// CONTROL related params
int control_timer_ms = 20;
Ticker control_timer;


void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    ledcSetup(servo_chan[0], SERVO_FREQ, SERVO_BIT);
    ledcAttachPin(servo_pin[0], servo_chan[0]);
    ledcSetup(servo_chan[1], SERVO_FREQ, SERVO_BIT);
    ledcAttachPin(servo_pin[1], servo_chan[1]);

    ledcSetup(motor_chan, MOTOR_FREQ, MOTOR_BIT);
    ledcAttachPin(motor_pin, motor_chan);

    control_timer.attach_ms(control_timer_ms, control_interrupt_func);
}

void loop()
{
    // put your main code here, to run repeatedly:
    float value;
    float value_5;float value_6;float value_7;float value_8;float val;

    value = constrain(ppm.rawChannelValue(1), 1000, 2000);
    value = fmap(value, 1000, 2000, -90.0, 90.0);
    servo_angle[0] = value + servo_central_angle[0];

    value = constrain(ppm.rawChannelValue(2), 1000, 2000);
    value = fmap(value, 1000, 2000, -90.0, 90.0);
    servo_angle[1] = value + servo_central_angle[1];

    value = constrain(ppm.rawChannelValue(3), 1000, 2000);
    //value = fmap(value, 1000, 2000, -2.0, 2.0);
    //motor_duty = value + motor_central_duty;
    
    value_5 = constrain(ppm.rawChannelValue(5), 1000, 2000);
    value_6 = constrain(ppm.rawChannelValue(6), 1000, 2000);
    value_7 = constrain(ppm.rawChannelValue(7), 1000, 2000);
    value_8 = constrain(ppm.rawChannelValue(8), 1000, 2000);
    
    
    if ( value_8 <=1500 ) //紧急停止
        {motor_duty = 10; }
    else if (value_8 >1500)
    {
      if ( value_7 > 1500)
      {

        //正转
          if (value_5 < 1200 && value_6 <1200)
          {
            value = fmap(value, 1000, 2000, 0, 0.5);
                motor_duty= value + motor_central_duty;
          }
          
          else if (value_5 < 1200 && value_6 >1200  && value_6 <1700)
                {motor_duty= 10.1;} 
          else if (value_5 < 1200 && value_6 >1700) {motor_duty= 10.15;}
                
          else if (value_5 > 1200 &&  value_5<1700  && value_6 <1200) {motor_duty= 10.2;}
                
          else if (value_5 > 1200 &&  value_5<1700  && value_6 >1200  && value_6 <1700) {motor_duty= 10.25;}
                
          else if (value_5 > 1200 &&  value_5<1700  && value_6 >1700) {motor_duty= 10.3;}
                
          else if (value_5 > 1700 && value_6 <1200) {motor_duty= 10.35;}
                
          else if (value_5 > 1700 && value_6 >1200  && value_6 <1700) {motor_duty= 10.4; }
                  
          else if (value_5 > 1700 && value_6 >1700) {motor_duty= 10.5;}
                
      }
    
      if ( value_7 < 1500)  
      {
        //反转
            if (value_5 < 1200 && value_6 <1200)
                {value = fmap(value, 2000, 1000, -0.5, 0);
                motor_duty= value + motor_central_duty;}
            if (value_5 < 1200 && value_6 >1200  && value_6 <1700)
                {motor_duty= 95;}
            else if (value_5 < 1200 && value_6 >1700)
                {motor_duty= 9.9;}
            else if (value_5 > 1200 &&  value_5<1700  && value_6 <1200)
                {motor_duty= 9.85;}
            else if (value_5 > 1200 &&  value_5<1700  && value_6 >1200  && value_6 <1700)
                {motor_duty= 9.8;}
            else if (value_5 > 1200 &&  value_5<1700  && value_6 >1700)
                {motor_duty= 9.75;}
            else if (value_5 > 1700 && value_6 <1200)
                {motor_duty= 9.7;}
            else if (value_5 > 1700 && value_6 >1200  && value_6 <1700)
                {motor_duty= 9.6;   }
            else if (value_5 > 1700 && value_6 >1700)
                {motor_duty= 9.5;}
      }
    }
    Serial.print(servo_angle[0]);
    Serial.print(',');
    Serial.print(servo_angle[1]);
    Serial.print(',');
    Serial.print(value);
    Serial.print(',');
    Serial.print(motor_duty);
    Serial.print(',');
    Serial.print(value_5);
    Serial.print(',');
    Serial.print(value_6);
    Serial.print(',');
    Serial.print(value_7);
    Serial.print(',');
    Serial.print(value_8);
    Serial.print('\n');
    
    delay(10);
}


void control_interrupt_func()
{
    ledcWrite(servo_chan[0], int(angle_to_duty(servo_angle[0]) * pow(2, SERVO_BIT) / 100.0));
    ledcWrite(servo_chan[1], int(angle_to_duty(servo_angle[1]) * pow(2, SERVO_BIT) / 100.0));

    ledcWrite(motor_chan, int(motor_duty * pow(2, MOTOR_BIT) / 100.0));
}

float angle_to_duty(float angle)
{
    float duty;

    duty = angle / 180.0 * 10.0 + 2.5;
    return duty;
}

float fmap(float val, float I_Min, float I_Max, float O_Min, float O_Max)
{
    return (((val - I_Min) * ((O_Max - O_Min) / (I_Max - I_Min))) + O_Min);
}
