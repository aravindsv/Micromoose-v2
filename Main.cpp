#include <stdio.h>
#include <stdint.h>
#include "STM32Library/Library/Library.h"
#include "Micromoose_Defines.h"

typedef SystickInterrupt<1000> systick;

int state = 0;

int leftEncoder = 0;
int rightEncoder = 0;
int leftEncoderChange = 0;
int rightEncoderChange = 0;
int encoderChange = 0;
int leftEncoderOld = 0;
int rightEncoderOld = 0;
int leftEncoderCount = 0;
int rightEncoderCount = 0;
int encoderCount = 0;
int distanceLeft = 0;

float curSpeedX = 0;
float curSpeedW = 0;

int targetSpeedX = 0;
int targetSpeedW = 0;

int encoderFeedbackX = 0;
int encoderFeedbackW = 0;

float posErrorX = 0;
float posErrorW = 0;
float oldPosErrorX = 0;
float oldPosErrorW = 0;

int posPwmX = 0;
int posPwmW = 0;

float pidInputX = 0;
float pidInputW = 0;
float kpX = .2, kiX = 0, kdX = 0.4;
float kpW = 0.01, kiW = 0, kdW = 0.02;
float kpW1 = 1;
float kdW1 = 26;
float kpW2 = 1;
float kdW2 = 36;

float accX = 1;
float decX = 1;
float accW = 1;
float decW = 1;

bool b_useIR = true;
bool b_useGyro = true;
bool b_useSpeedProfile = true;
bool onlyUseGyroFeedback = true;
bool onlyUseEncoderFeedback = false;

int moveSpeed = 20;
int turnSpeed = 20;
int returnSpeed = 20;
int stopSpeed = 10;
int maxSpeed = 60;

int gyroFeedbackRatio = 5;
int a_scale = 1;
int ld_middleValue = 0;
int rd_middleValue = 0;
int ir_sensorError = 0;

unsigned int ld_val = 0, lf_val = 0, rf_val = 0, rd_val = 0;
int gyro_val = 0;
float battery_voltage = 0;

void readVoltageMeter()
{
    battery_voltage = battery::readIn(3)*3;
}

void readIRSensors()
{
    ld_emitter::set(0);
    ld_val = receivers::readIn(8);
    ld_emitter::set(1);
    delay_us_hard(6);
    ld_val = receivers::readIn(8) - ld_val;
    ld_emitter::set(0);
    delay_us_hard(8);

    lf_emitter::set(0);
    lf_val = receivers::readIn(9);
    lf_emitter::set(1);
    delay_us_hard(6);
    lf_val = receivers::readIn(9) - lf_val;
    lf_emitter::set(0);
    delay_us_hard(8);

    rf_emitter::set(0);
    rf_val = receivers::readIn(5);
    rf_emitter::set(1);
    delay_us_hard(6);
    rf_val = receivers::readIn(5) - rf_val;
    rf_emitter::set(0);
    delay_us_hard(8);

    rd_emitter::set(0);
    rd_val = receivers::readIn(4);
    rd_emitter::set(1);
    delay_us_hard(6);
    rd_val = receivers::readIn(4) - rd_val;
    rd_emitter::set(0);
    delay_us_hard(8);

    if (ld_val > ld_middleValue && rd_val < rd_middleValue)
    {
        ir_sensorError = ld_middleValue - ld_val;
    }
    else if (rd_val > rd_middleValue && ld_val < ld_middleValue)
    {
        ir_sensorError = rd_val - rd_middleValue;
    }
    else
    {
        ir_sensorError = 0;
    }
}

int gyroVal = 0;
int gyroOffset = 12;
void readGyro()
{
    gyroVal = gyro::readIn(15)-gyro::readIn(14)-gyroOffset;
}

void getEncoderStatus()
{
    leftEncoder = l_enc::readEncoder();
    rightEncoder = r_enc::readEncoder();

    leftEncoderChange = leftEncoder - leftEncoderOld;
    rightEncoderChange = rightEncoder - rightEncoderOld;
    encoderChange = (leftEncoderChange+rightEncoderChange)/2;

    leftEncoderOld = leftEncoder;
    rightEncoderOld = rightEncoder;

    leftEncoderCount += leftEncoderChange;
    rightEncoderCount += rightEncoderChange;
    encoderCount = (leftEncoderCount+rightEncoderCount)/2;

    distanceLeft -= encoderChange;
}

void updateCurrentSpeed()
{
    if (curSpeedX < targetSpeedX)
    {
        curSpeedX += accX;
        if (curSpeedX > targetSpeedX)
        {
            curSpeedX = targetSpeedX;
        }
    }
    else if (curSpeedX > targetSpeedX)
    {
        curSpeedX -= decX;
        if (curSpeedX < targetSpeedX)
        {
            curSpeedX = targetSpeedX;
        }
    }
    if (curSpeedW < targetSpeedW)
    {
        curSpeedW += accW;
        if (curSpeedW > targetSpeedW)
        {
            curSpeedW = targetSpeedW;
        }
    }
    else if (curSpeedW > targetSpeedW)
    {
        curSpeedW -= decW;
        if (curSpeedW < targetSpeedW)
        {
            curSpeedW = targetSpeedW;
        }
    }

}

    int leftBaseSpeed;
    int rightBaseSpeed;

void calculateMotorPwm()
{
    int gyroFeedback;
    int rotationalFeedback;
    int sensorFeedback;

    encoderFeedbackX = rightEncoderChange + leftEncoderChange;
    encoderFeedbackW = rightEncoderChange - leftEncoderChange;

    gyroFeedback = gyroVal*gyroFeedbackRatio;
    sensorFeedback = ir_sensorError/a_scale;

    if (onlyUseGyroFeedback)
    {
        rotationalFeedback = gyroFeedback;
    }
    else if (onlyUseEncoderFeedback)
    {
        rotationalFeedback = encoderFeedbackW;
    }
    else
    {
        rotationalFeedback = encoderFeedbackW + gyroFeedback;
    }

    posErrorX += curSpeedX - encoderFeedbackX;
    posErrorW += curSpeedW - rotationalFeedback;
    if (posErrorX > 100) posErrorX = 100;
    else if (posErrorX < -100) posErrorX = -100;
    if (posErrorW > 100) posErrorW = 100;
    else if (posErrorW < -100) posErrorW = -100;

    posPwmX = (kpX * posErrorX) + (kdX * (posErrorX - oldPosErrorX));
    posPwmW = (kpW * posErrorW) + (kdW * (posErrorW - oldPosErrorW));

    oldPosErrorX = posErrorX;
    oldPosErrorW = posErrorW;

    leftBaseSpeed = posPwmX - posPwmW;
    rightBaseSpeed = posPwmX + posPwmW;

    if (leftBaseSpeed > maxSpeed)
    {
        leftBaseSpeed = maxSpeed;
    }
    else if (leftBaseSpeed < -1*maxSpeed)
    {
        leftBaseSpeed = -1*maxSpeed;
    }
    if (rightBaseSpeed > maxSpeed)
    {
        rightBaseSpeed = maxSpeed;
    }
    else if (rightBaseSpeed < -1*maxSpeed)
    {
        rightBaseSpeed = -1*maxSpeed;
    }

    setLeftMotor(leftBaseSpeed);
    setRightMotor(rightBaseSpeed);

}

void speedProfile()
{
    getEncoderStatus();
    updateCurrentSpeed();
    calculateMotorPwm();
}

char btnString[50];
//Systick User Code
void Systick_User(void)
{   
    readVoltageMeter();
    if (b_useIR)
    {
        readIRSensors();
    }

    if (b_useGyro)
    {
        readGyro();
    }
    if (b_useSpeedProfile)
    {
        speedProfile();
    }
    if (milliseconds > 1000 && milliseconds % 500 == 0)
    {
        sprintf(btnString, "Left: %d/%d\tRight: %d/%d\r\n", leftEncoder, l_enc::readEncoder(), rightEncoder, r_enc::readEncoder());
        serial::sendString(btnString);
    }
}

//Button 1 Handler
int oldMillis = 0;
int debounceThresh = 1;
extern "C" void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & (1<<13)) {
        //Btn1
        serial::sendString("State Changed\r\n");
        state += 1;

        //Clear interrupt mask
        EXTI->PR |= (1<<13);
    }
}



//Button 2 Handler
extern "C" void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & (1<<5)) {
        //Btn2
        serial::sendString("Button 2 was pressed\r\n");

        //Clear interrupt mask
        EXTI->PR |= (1<<5);
    }
}

int turning = 0;

void forwardDistance(int distance, int speed, bool coast)
{
    distanceLeft = distance;
    while(distanceLeft > speed/decX)
    {
        targetSpeedX = speed;
        targetSpeedW = 0;
    }
    if (!coast)
    {
        targetSpeedX = 0;
    }
}


int main(int argc, char** argv) 
{
    initMicromoose();
    serial::sendString("Serial initialized...");
    systick::initSysTick();
    serial::sendString("SysTick initialized\r\n");
    state = 0;
    char printString[75];
    serial::sendString("State = 0\r\n");
    playTone(1200, 250);
    int IRThresh = 3000;
    while(1)
    {
        targetSpeedX = 0;
        targetSpeedW = 0;

        if (rd_val > IRThresh) led4::set(1);
        else led4::set(0);
        if (rf_val > IRThresh) led3::set(1);
        else led3::set(0);
        if (lf_val > IRThresh) led2::set(1);
        else led2::set(0);
        if (ld_val > IRThresh) led1::set(1);
        else led1::set(0);
        // if (rf_val > 10000 && rd_val > 10000 && ld_val > 10000 && lf_val > 10000)
        // {
        //     playTone(1100, 50);
        //     delay_ms(100);
        //     if (rf_val < 1000 && rd_val < 1000)// && ld_val < 1000 && lf_val < 1000) 
        //     {
        //         playTone(1500, 50);
        //         delay_ms(10);
        //         playTone(1700, 50);
        //         delay_ms(10);
        //         playTone(1800, 50);
        //         delay_ms(10);
        //         playTone(1850, 50);
        //         delay_ms(10);
        //         playTone(1875, 50);
        //         delay_ms(10);
        //     }
        // }

        // if (state != 0)
        // {
        //     playTone(1000, 50);
        //     delay_ms(50);
        //     playTone(1480, 50);
        //     delay_ms(500);
        //     forwardDistance(CountsForDistance(180), 10, true);
        //     forwardDistance(CountsForDistance(180), 10, false);
        //     // targetSpeedX = 0;
            
        //     // int startL = l_enc::readEncoder();
        //     // int startR = r_enc::readEncoder();
        //     // while(l_enc::readEncoder() - startL != 1780 || r_enc::readEncoder() - startR != 1780)
        //     // {
        //     //     if (l_enc::readEncoder() - startL < 1780)
        //     //     {
        //     //         setLeftMotor(3);
        //     //     }
        //     //     else if (l_enc::readEncoder() - startL > 1780)
        //     //     {
        //     //         setLeftMotor(0);
        //     //     }
        //     //     if (r_enc::readEncoder() - startR < 1780)
        //     //     {
        //     //         setRightMotor(3);
        //     //     }
        //     //     else if (r_enc::readEncoder() - startR > 1780)
        //     //     {
        //     //         setRightMotor(0);
        //     //     }
        //     // }
        //     // setLeftMotor(0);
        //     // setRightMotor(0);

        //     playTone(1480, 50);
        //     delay_ms(50);
        //     playTone(1000, 50);
        //     delay_ms(500);
        // }

        state = 0;
    }
    

    // // Use this to test the duration of SysTick if it goes awry again
    // while(1)
    // {
    //     led1::set(1);
    //     delay_ms(500);
    //     led1::set(0);
    //     delay_ms(500);
    // }
}








