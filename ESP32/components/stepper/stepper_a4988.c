#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <esp_timer.h>
#include <driver/gpio.h>

static const char *StepperTAG = "Stepper Driver";

// Timer
esp_timer_handle_t timer_handle;
volatile uint16_t INTERVAL_US = 2000;

// definere stepper variables
gpio_config_t stepper_gpio_conf;

typedef struct StepperInfo {
    // a4988 pins
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t ms1_pin;
    uint8_t ms2_pin;
    uint8_t ms3_pin;
    // stepper variables
    float acceleration;
    volatile uint8_t step_res; // step size resolution (microstepping)
    volatile uint16_t targetPos; // steps that the stepper have to perform
    volatile uint16_t currentPos; // steps done by the stepper
    volatile uint8_t step_direction; // what direction the stepper is rotating
    volatile float speed; // current speed interval in US
    volatile float last_speed; // last speed interval in US
    volatile float max_speed; // the minium interval before the stepper resonate in US
     /// The current interval between steps in microseconds.
    /// 0 means the motor is currently stopped with _speed == 0
    volatile uint16_t step_interval;
    volatile int16_t n; // The step counter for speed calculations
    volatile uint16_t cn; // Last step size in microseconds
    volatile uint16_t c0; // Initial step size in microseconds
    /// Min step size in microseconds based on maxSpeed
    volatile uint16_t cmin; // at max speed
} StepperInfo;

// antal stepper i ens kredsløb
#define NUM_STEPPERS 1

/* Function declarations */
int max(int num1, int num2)
{
    return (num1 > num2 ) ? num1 : num2;
}

volatile StepperInfo steppers[NUM_STEPPERS];

void call_stepper()
{
    ESP_LOGI(StepperTAG, "Hello from Stepper driver");
}

void stepper_setMircostepping(uint8_t motor_num ,uint8_t res){
    ESP_LOGI(StepperTAG, "Stepper divider:%d", res);
    switch (res)
    {
        case 2: // Half step
            gpio_set_level(steppers[motor_num].ms1_pin, 1);
            gpio_set_level(steppers[motor_num].ms2_pin, 0);
            gpio_set_level(steppers[motor_num].ms3_pin, 0);
            break;

        case 4: // 1/4 step
            gpio_set_level(steppers[motor_num].ms1_pin, 0);
            gpio_set_level(steppers[motor_num].ms2_pin, 1);
            gpio_set_level(steppers[motor_num].ms3_pin, 0);
            break;

        case 8: // 1/8 step
            gpio_set_level(steppers[motor_num].ms1_pin, 1);
            gpio_set_level(steppers[motor_num].ms2_pin, 1);
            gpio_set_level(steppers[motor_num].ms3_pin, 0);
            break;

        case 16: // 1/16 step
            gpio_set_level(steppers[motor_num].ms1_pin, 1);
            gpio_set_level(steppers[motor_num].ms2_pin, 1);
            gpio_set_level(steppers[motor_num].ms3_pin, 1);
            break;

        default:
            gpio_set_level(steppers[motor_num].ms1_pin, 0);
            gpio_set_level(steppers[motor_num].ms2_pin, 0);
            gpio_set_level(steppers[motor_num].ms3_pin, 0);

    }
}

// ISP loop function
static void stepper_timer_callback(void* arg)
{
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        if(steppers[i].currentPos < steppers[i].targetPos){
            gpio_set_level(steppers[i].dir_pin, steppers[i].step_direction); // sets DIR arro
            if (steppers[i].step_res != 0)
            {
                stepper_setMircostepping(i, steppers[i].step_res);
                steppers[i].step_res = 0;

            }   
            // makes step 
            gpio_set_level(steppers[i].step_pin, 1); // sets STEP pin HIGH
            gpio_set_level(steppers[i].step_pin, 0); // sets STEP pin LOW
            steppers[i].currentPos++;
        } else {
            gpio_set_level(steppers[i].step_pin, 0);
        }
    }
}

// sets the interval of the timer and updates it.
void stepper_setSpeed(uint16_t new_timer_period) 
{

    INTERVAL_US = new_timer_period;
    // Restarts the timer with new timer interval
    esp_timer_restart(timer_handle, INTERVAL_US);
}

// calculates the distance to target postion
uint8_t distanceToGo(uint8_t motor_num)
{
    return steppers[motor_num].targetPos - steppers[motor_num].currentPos;
}

// calculate new step interval for stepper
void computeNewSpeed(uint8_t motor_num)
{
    long distanceTo = distanceToGo(motor_num); // +ve is clockwise from curent location

    long stepsToStop = (long)((steppers[motor_num].speed * steppers[motor_num].speed) / (2.0 * steppers[motor_num].acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
	// We are at the target and its time to stop
	steppers[motor_num].speed = 0;
	steppers[motor_num].speed = 0;
	steppers[motor_num].n = 0;
	return;
    }
        if (distanceTo > 0)
    {

        
	// We are anticlockwise from the target
	// Need to go clockwise from here, maybe decelerate now
	if (steppers[motor_num].n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= distanceTo) || steppers[motor_num].dir_pin == 1)
        {
		    steppers[motor_num].n = -stepsToStop; // Start deceleration
        }
	}
	else if (steppers[motor_num].n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < distanceTo) && steppers[motor_num].dir_pin == 0)
        {
		    steppers[motor_num].n = -steppers[motor_num].n; // Start accceleration
        }
	}
    }
    else if (distanceTo < 0)
    {
	// We are clockwise from the target
	// Need to go anticlockwise from here, maybe decelerate
	if (steppers[motor_num].n > 0)
	{
	    // Currently accelerating, need to decel now? Or maybe going the wrong way?
	    if ((stepsToStop >= -distanceTo) || steppers[motor_num].dir_pin == 0)
        {
		    steppers[motor_num].n = -stepsToStop; // Start deceleration
        }
	}
	else if (steppers[motor_num].n < 0)
	{
	    // Currently decelerating, need to accel again?
	    if ((stepsToStop < -distanceTo) && steppers[motor_num].dir_pin == 1)
		{
            steppers[motor_num].n = -steppers[motor_num].n; // Start accceleration
        }
	}
    }

    // Need to accelerate or decelerate
    if (steppers[motor_num].n == 0)
    {
	    // First step from stopped
	    steppers[motor_num].cn = steppers[motor_num].c0;
	    steppers[motor_num].step_direction = (distanceTo > 0) ? 0 : 1;
    }
    else
    {
	    // Subsequent step. Works for accel (n is +ve) and decel (n is -ve).
	    steppers[motor_num].cn = steppers[motor_num].cn - ((2.0 * steppers[motor_num].cn) / ((4.0 * steppers[motor_num].n) + 1)); // Equation 13
	    steppers[motor_num].cn = max(steppers[motor_num].cn, steppers[motor_num].cmin); 
    }
    
    steppers[motor_num].n++;
    steppers[motor_num].step_interval = steppers[motor_num].cn;
    steppers[motor_num].speed = 1000000.0 / steppers[motor_num].cn;
    if (steppers[motor_num].step_direction == 1)
    {
	    steppers[motor_num].speed = -steppers[motor_num].speed;
    }

}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
// need run function


// move micro stepper without linear motion
void stepper_moveMicrostep(uint8_t motor_num, uint16_t steps,  uint8_t direction, uint8_t stepping_resolution) {
    ESP_LOGI(StepperTAG, "steps runing");
    // motor 1A steps
    steppers[motor_num-1].targetPos = steps;
    steppers[motor_num-1].currentPos = 0;
    steppers[motor_num-1].step_direction = direction;
    steppers[motor_num-1].step_res = stepping_resolution;
}

// Funktion der tager bevæger en motor et bestemt antal steps i en retning
void stepper_moveStep(uint8_t motor_num, uint16_t steps,  uint8_t direction) {
    ESP_LOGI(StepperTAG, "steps runing");
    // motor 1A steps
    steppers[motor_num-1].targetPos = steps;
    steppers[motor_num-1].currentPos = 0;
    steppers[motor_num-1].step_direction = direction;
}

void stepper_config_reset()
{
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i].step_pin = 0;
        steppers[i].dir_pin = 0;
        steppers[i].currentPos = 0;
        steppers[i].step_direction = 0;
        steppers[i].max_speed = 0;   
    }
}

void stepper_config(uint8_t step_num, uint8_t step_pin, uint8_t dir_pin)
{
    uint8_t i = step_num-1;
    steppers[i].step_pin = step_pin;
    steppers[i].dir_pin = dir_pin;
    steppers[i].currentPos = 0;
    steppers[i].step_direction = 1;
    steppers[i].targetPos = 0;
    steppers[i].max_speed = 1400;

    stepper_gpio_conf.intr_type = GPIO_INTR_DISABLE;
    stepper_gpio_conf.mode = GPIO_MODE_OUTPUT;
    // configures all step and dir pins
    stepper_gpio_conf.pin_bit_mask |= (1ULL<<steppers[i].step_pin) | (1ULL<<steppers[i].dir_pin);
    stepper_gpio_conf.pull_down_en = 0;
    stepper_gpio_conf.pull_up_en = 0;

    gpio_config(&stepper_gpio_conf);
    //NUM_STEPPERS++;
}

void microstepping_config(uint8_t ms1_pin, uint8_t ms2_pin, uint8_t ms3_pin)
{
    for(int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i].ms1_pin = ms1_pin;
        steppers[i].ms2_pin = ms2_pin;
        steppers[i].ms3_pin = ms3_pin;   
    }

    stepper_gpio_conf.intr_type = GPIO_INTR_DISABLE;
    stepper_gpio_conf.mode = GPIO_MODE_OUTPUT;
    // configures all step and dir pins
    stepper_gpio_conf.pin_bit_mask |= (1ULL<<ms1_pin) | (1ULL<<ms2_pin) | (1ULL<<ms3_pin);
    stepper_gpio_conf.pull_down_en = 0;
    stepper_gpio_conf.pull_up_en = 0;

    gpio_config(&stepper_gpio_conf);
}

void stepper_start_timer()
{
    // Create the timer
    esp_timer_create_args_t timer_conf = {
        .callback = &stepper_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "stepper_timer"
    };

    esp_timer_create(&timer_conf, &timer_handle);

    // Start the timer
    esp_timer_start_periodic(timer_handle, INTERVAL_US);
}

void stepper_Init()
{
    stepper_start_timer();
    ESP_LOGI(StepperTAG, "All Pins Init");
}