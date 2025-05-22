/* ******************************************************************************************************* */
/**
 * This is an Arduino library for WEMOS Motor Shield
 * (https://wiki.wemos.cc/products:d1_mini_shields:motor_shield).
 * written by : danielfmo
 * source code: https://github.com/danielfmo/WEMOS_Motor_Shield_Arduino_Library
 *
 * This library does implement a reworked version of the original command messages, thus needing
 * a specific version of the firmware available here: https://github.com/danielfmo/wemos_motor_shield
 *
 * This file is part of the WEMOS_Motor_Shield_Arduino_Library
 * Which is release under The MIT License (MIT)
 * Please see LICENSE.md file for details
 *
 * Motor Shield I2C Address:
 * The shield has a matrix in PCB that can be soldered in order to set the shield Address.
 * AD0 | AD1 | Address | MOTORSHIELD_ADDRESS enum
 *  -  | -   | 0x2D    | MOTORSHIELD_AD00
 *  -  | x   | 0x2E    | MOTORSHIELD_AD01
 *  x  | -   | 0x2F    | MOTORSHIELD_AD10
 *  x  | x   | 0x30    | MOTORSHIELD_AD11
 * Where 'x' means that the jumper is soldered and '-' means that the jumper is kept open.
 *
 * Motor Shield PWM resolution and max frequency:
 * The STM32F030 MCU present on the shield have a limitation on the PWM generation, where the PWM frequency
 * times the PWM steps cannot exceed the CPU frequency (8MHz). This means that there is a maximum possible
 * frequency for a given PWM resolution.
 * PWM steps | PWM max frequency | MOTORSHIELD_PWM_RESOLUTION enum
 *        64 |       65'535 (Hz) | MOTORSHIELD_PWM_RES_64STEP or MOTORSHIELD_PWM_RES_6BIT
 *       128 |       62'500 (Hz) | MOTORSHIELD_PWM_RES_128STEP or MOTORSHIELD_PWM_RES_7BIT
 *       256 |       31'250 (Hz) | MOTORSHIELD_PWM_RES_256STEP or MOTORSHIELD_PWM_RES_8BIT
 *       512 |       15'625 (Hz) | MOTORSHIELD_PWM_RES_512STEP or MOTORSHIELD_PWM_RES_9BIT
 *      1024 |        7'812 (Hz) | MOTORSHIELD_PWM_RES_1024STEP or MOTORSHIELD_PWM_RES_10BIT
 *      2048 |        3'906 (Hz) | MOTORSHIELD_PWM_RES_2048STEP or MOTORSHIELD_PWM_RES_11BIT
 *      4096 |        1'953 (Hz) | MOTORSHIELD_PWM_RES_4096STEP or MOTORSHIELD_PWM_RES_12BIT
 *
 * Motor Shield Motor Direction:
 * The shield accepts 5 'directions' to set the motor.
 *         DIRECTION |     MOTORSHIELD_ADDRESS | Description
 *    Brake or Short |   MOTOR_DIRECTION_BRAKE | Stops the motor by shorting connectors (Active brake)
 * Counter-Clockwork |     MOTOR_DIRECTION_CCW | Turns the motor on Counter-Clockwork direction
 *         Clockwork |      MOTOR_DIRECTION_CW | Turns the motor on Clockwork direction
 *     Coast or Open |   MOTOR_DIRECTION_COAST | Stops the motor by opening the connectors, coast.
 *           Standby | MOTOR_DIRECTION_STANDBY | Sets the TB6612FNG on standby mode, impacts all motors.
 *
 */
/* ******************************************************************************************************* */

#include "wms.h"

/* ******************************************************************************************************* */
MOTORSHIELD_ADDRESS WMS_address;
bool WMS_sendCommandSetMotor(MOTORSHIELD_MOTOR_NUMBER mot, MOTORSHIELD_MOTOR_DIRECTION direction, uint16_t pwm_value);
bool WMS_sendCommandConfigurePWM(MOTORSHIELD_PWM_RESOLUTION _pwm_resolution, uint16_t _pwm_freqency);

void DriveA(int32_t pwm) 
{
    MOTORSHIELD_MOTOR_DIRECTION direction = MOTOR_DIRECTION_COAST;
    if (pwm > 0) {
        direction = MOTOR_DIRECTION_CW;
    } else if (pwm < 0) {
        direction = MOTOR_DIRECTION_CCW;
    }
    uint16_t pwm_value = abs(pwm);
	WMS_sendCommandSetMotor(MOTORSHIELD_MOTORA,direction,pwm_value); 
}

void DriveB(int32_t pwm) 
{
    MOTORSHIELD_MOTOR_DIRECTION direction = MOTOR_DIRECTION_COAST;
    if (pwm > 0) {
        direction = MOTOR_DIRECTION_CW;
    } else if (pwm < 0) {
        direction = MOTOR_DIRECTION_CCW;
    }
    uint16_t pwm_value = abs(pwm);
	WMS_sendCommandSetMotor(MOTORSHIELD_MOTORB,direction,pwm_value); 
}

void DriveAB(int32_t pwm_motorA, int32_t pwm_motorB)
{
	DriveA(pwm_motorA);
  //delayMicroseconds(2);
	DriveB(pwm_motorB);
}

/**
 * brake()
 * brakes "short" motor
 */
void BrakeA() 
{ 
	WMS_sendCommandSetMotor(MOTORSHIELD_MOTORA, MOTOR_DIRECTION_BRAKE,0); 
}

void BrakeB() 
{ 
	WMS_sendCommandSetMotor(MOTORSHIELD_MOTORB, MOTOR_DIRECTION_BRAKE,0); 
}
void BrakeAB() 
{ 
	WMS_sendCommandSetMotor(MOTORSHIELD_MOTORA, MOTOR_DIRECTION_BRAKE,0); 
	WMS_sendCommandSetMotor(MOTORSHIELD_MOTORB, MOTOR_DIRECTION_BRAKE,0); 
}


/**
 * coast()
 * stops "coast" motor
 */
void CoastA() 
{
	WMS_sendCommandSetMotor(MOTORSHIELD_MOTORA, MOTOR_DIRECTION_COAST,0); 	
}

void CoastB() 
{
	WMS_sendCommandSetMotor(MOTORSHIELD_MOTORB, MOTOR_DIRECTION_COAST,0); 	
}
void CoastAB() 
{
	WMS_sendCommandSetMotor(MOTORSHIELD_MOTORA, MOTOR_DIRECTION_COAST,0); 
	WMS_sendCommandSetMotor(MOTORSHIELD_MOTORB, MOTOR_DIRECTION_COAST,0); 	
}

/**
 * _sendCommandSetMotor()
 *  @param  direction - direction of the motor (Brake, CW, CCW, Stop, Standby, Coast)
 *  @param  pwm_value - PWM pulse for motor
 * Sets the motor's PWM duty cycle / pulse and direction
 * |       0001 |      4 bit |     4 bit |        12 bit |
 * |  set motor |      Motor | Direction |          Step |
 * |       0001 |       0001 |      0001 | 0010 00000000 | -> Set MotorB at step 512
 */
bool WMS_sendCommandSetMotor(MOTORSHIELD_MOTOR_NUMBER mot, MOTORSHIELD_MOTOR_DIRECTION direction, uint16_t pwm_value)
 {
    Wire.beginTransmission(WMS_address);
    Wire.write((byte)mot | (byte)0x10);
    Wire.write((byte)(direction << 4) | ((byte)(pwm_value >> 8) & (byte)0x0F));
    Wire.write((byte)pwm_value);
    if (Wire.endTransmission(true) != 0) {
        return false;
    }
    return true;
}

/**
 * begin()
 * Initializes I2C and configures the motor shield, call this function before doing anything else
 * @return (int) - error code:
 *     - 0 success
 *     - 1 data too long to fit in transmit data16
 *     - 2 received NACK on transmit of address
 *     - 3 received NACK on transmit of data
 *     - 4 other error
 */
 
bool WMS_Begin(MOTORSHIELD_ADDRESS address, MOTORSHIELD_PWM_RESOLUTION pwm, uint16_t freq) 
{
    Wire.begin();
    Wire.setClock(400000);
    WMS_address = address;
     if (WMS_sendCommandConfigurePWM(pwm,freq) == false) 
     {
         return false;
     }
	CoastAB();
    //brake();
    //setStandby();
    return true;
}


/**
 * _sendCommandConfigurePWM()
 * Sets the MotorShield's PWM resolution and frequency
 * |  4 bit CMD |      4 bit |                    16 bit |
 * | config pwm | Resolution |             PWM Frequency |
 * |       0000 |       1010 |         00010011 10001000 | -> Set 10 bit resolution = 1024 steps and 5KHz Frequency
 */
bool WMS_sendCommandConfigurePWM(MOTORSHIELD_PWM_RESOLUTION _pwm_resolution, uint16_t _pwm_freqency) 
{
    Wire.beginTransmission(WMS_address);
    Wire.write(((byte)(_pwm_resolution)) & (byte)0x0F);
    Wire.write((byte)(_pwm_freqency >> 8));
    Wire.write((byte)_pwm_freqency);
    if (Wire.endTransmission(true) != 0) {
        return false;
    }
    return true;
}
/* ******************************************************************************************************* */
