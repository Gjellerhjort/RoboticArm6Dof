#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <esp_timer.h>
#include <driver/gpio.h>

// Timer
esp_timer_handle_t timer_handle;
volatile uint16_t INTERVAL_US = 2000;

// definere stepper variables
gpio_config_t stepper_gpio_conf;

volatile uint8_t NUM_STEPPERS = 0;

typedef struct StepperInfo {
    // Stepper Pins
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t ms1_pin;
    uint8_t ms2_pin;
    uint8_t ms3_pin;

    // derived parameters
    unsigned int c0;                // step interval for first step, determines acceleration
    long stepPosition;              // current position of stepper (total of all movements taken so far)

    volatile uint8_t step_res; // step size resolution (microstepping)
    volatile uint16_t step_count; // number of steps completed in current movement
    volatile uint16_t total_steps; // number of steps requested for current movement
    volatile bool movementDone;
    volatile uint8_t step_direction; // what direction the motor is spinning 
    volatile uint16_t min_interval; // min interval between steps 

    float acceleration;

    volatile unsigned int rampUp_step_count;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)
    volatile unsigned long estStepsToSpeed;  // estimated steps required to reach max speed
    volatile unsigned long estTimeForMove;   // estimated time (interrupt ticks) required to complete movement
    volatile unsigned long rampUpStepTime;
    volatile float speedScale;               // used to slow down this motor to make coordinated movement with other motors

    // per iteration variables (potentially changed every interrupt)
    volatile unsigned int n;                 // index in acceleration curve, used to calculate next interval
    volatile float d;                        // current interval length
    volatile unsigned long di;               // above variable truncated
    volatile unsigned int stepCount;         // number of steps completed in current movement
} StepperInfo;

volatile StepperInfo steppers[1];
volatile uint8_t remainingSteppersFlag = 0;

void call_stepper()
{
    ESP_LOGI("Stepper Driver" ,"Hello from Stepper driver");
}

volatile uint8_t nextStepperFlag = 0;

void setNextInterruptInterval() {

  bool movementComplete = true;

  unsigned long mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! steppers[i].movementDone )
      movementComplete = false;
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    esp_timer_stop(timer_handle);
  }
  esp_timer_restart(timer_handle, mind / portTICK_PERIOD_MS*10);
}


void stepper_setMircostepping(uint8_t motor_num ,uint8_t res)
{
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


// Stepper Timer control loop
static void stepper_timer_callback(void* arg)
{
    long long unsigned int period;
    esp_timer_get_period(timer_handle, &period);
    long unsigned int tmpCtr = period;
    
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        if ( ! ((1 << i) & remainingSteppersFlag) ) {
            continue;
        }
        if ( ! (nextStepperFlag & (1 << i)) ) {
            steppers[i].di -= tmpCtr;
            continue;
        }
        if(steppers[i].step_count < steppers[i].total_steps){
            if (steppers[i].step_res != 0)
            {
                stepper_setMircostepping(i, steppers[i].step_res);
                steppers[i].step_res = 0;

            }
            // makes step 

            gpio_set_level(steppers[i].step_pin, 1); // sets STEP pin HIGH
            gpio_set_level(steppers[i].step_pin, 0); // sets STEP pin LOW
            steppers[i].step_count++;
            steppers[i].stepPosition += steppers[i].step_direction;
            if (steppers[i].step_count >= steppers[i].total_steps) {
                steppers[i].movementDone = true;
                remainingSteppersFlag &= ~(1 << i);
            }
        }

        if ( steppers[i].rampUp_step_count == 0 ) {
            steppers[i].n++;
            steppers[i].d = steppers[i].d - (2 * steppers[i].d) / (4 * steppers[i].n + 1);
            if ( steppers[i].d <= steppers[i].min_interval ) {
                steppers[i].d = steppers[i].min_interval;
                steppers[i].rampUp_step_count = steppers[i].step_count;
            }
            if ( steppers[i].step_count >= steppers[i].total_steps / 2 ) {
                steppers[i].rampUp_step_count = steppers[i].step_count;
            }
            steppers[i].rampUpStepTime += steppers[i].d;
        }
        else if ( steppers[i].step_count >= steppers[i].total_steps - steppers[i].rampUp_step_count ) {
            steppers[i].d = (steppers[i].d * (4 * steppers[i].n + 1)) / (4 * steppers[i].n + 1 - 2);
            steppers[i].n--;
        }

        steppers[i].di = steppers[i].d * steppers[i].speedScale; // integer
    }

    setNextInterruptInterval();

}

void stepper_setSpeed(uint16_t new_timer_period) 
{

    INTERVAL_US = new_timer_period;
    // Restarts the timer with new timer interval
    esp_timer_restart(timer_handle, INTERVAL_US);
}   

float getDurationOfAcceleration(uint8_t motor_num, unsigned int numSteps) {
  float d = steppers[motor_num].c0;
  float totalDuration = 0;
  for (unsigned int n = 1; n < numSteps; n++) {
    d = d - (2 * d) / (4 * n + 1);
    totalDuration += d;
  }
  return totalDuration;
}

void reset_stepper(uint8_t motor_num)
{
    steppers[motor_num].c0 = steppers[motor_num].acceleration;
    steppers[motor_num].d = steppers[motor_num].c0;
    steppers[motor_num].di = steppers[motor_num].d;
    steppers[motor_num].step_count = 0;
    steppers[motor_num].movementDone = false;
    steppers[motor_num].speedScale = 1;

    float a = steppers[motor_num].min_interval / (float)steppers[motor_num].c0;
    a *= 0.676;

    float m = ((a*a - 1) / (-2 * a));
    float n = m * m;

    steppers[motor_num].estStepsToSpeed = n;
}

void stepper_create_timer()
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

void adjustSpeedScales() {
  float maxTime = 0;
  
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;
    if ( steppers[i].estTimeForMove > maxTime )
      maxTime = steppers[i].estTimeForMove;
  }

  if ( maxTime != 0 ) {
    for (int i = 0; i < NUM_STEPPERS; i++) {
      if ( ! ( (1 << i) & remainingSteppersFlag) )
        continue;
      steppers[i].speedScale = maxTime / steppers[i].estTimeForMove;
    }
}
}

void runAndWait() {
  stepper_create_timer();
  adjustSpeedScales();
  setNextInterruptInterval();

  while ( remainingSteppersFlag );
  remainingSteppersFlag = 0;
  nextStepperFlag = 0;
}

void prepareMovement(uint8_t motor_num, uint16_t steps)
{
    gpio_set_level(steppers[motor_num].dir_pin, steps < 0 ? 1 : 0); // sets DIR arro
    steppers[motor_num].step_direction = steps > 0 ? 1 : -1;
    steppers[motor_num].total_steps = steps;
    reset_stepper(motor_num);

    remainingSteppersFlag |= (1 << motor_num);

    unsigned long stepsAbs = abs(steps);

    if ((2 * steppers[motor_num].estStepsToSpeed) < stepsAbs ) { // there will be a period of time at full speed
        unsigned long stepsAtFullSpeed = stepsAbs - 2 * steppers[motor_num].estStepsToSpeed;
        float accelDecelTime = getDurationOfAcceleration(motor_num, steppers[motor_num].estStepsToSpeed);
        steppers[motor_num].estTimeForMove = 2 * accelDecelTime + stepsAtFullSpeed * steppers[motor_num].min_interval;
    }
    else {    // will not reach full speed before needing to slow down again
        float accelDecelTime = getDurationOfAcceleration(motor_num, stepsAbs / 2 );
        steppers[motor_num].estTimeForMove = 2 * accelDecelTime;
    }
}

void stepper_moveMicrostep(uint8_t motor_num, uint16_t steps,  uint8_t direction, uint8_t stepping_resolution) {
    ESP_LOGI("stepper driver", "steps runing");
    // motor 1A steps
    steppers[motor_num-1].total_steps = steps;
    steppers[motor_num-1].step_count = 0;
    steppers[motor_num-1].step_direction = direction;
    steppers[motor_num-1].step_res = stepping_resolution;
}

// Funktion der tager bevæger en motor et bestemt antal steps i en retning
void stepper_moveStep(uint8_t motor_num, uint16_t steps,  uint8_t direction) {
    ESP_LOGI("stepper driver", "steps runing");
    // motor 1A steps
    steppers[motor_num-1].total_steps = steps;
    steppers[motor_num-1].step_direction = direction;
    steppers[motor_num-1].step_res = 0;
}

void stepper_config_reset()
{
    for (int i = 0; i < NUM_STEPPERS; i++)
    {
        steppers[i].step_pin = 0;
        steppers[i].dir_pin = 0;
        steppers[i].step_count = 0;
        steppers[i].step_direction = 0;
        steppers[i].total_steps = 0;
        steppers[i].min_interval = 0;   
    }
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

void stepper_config(uint8_t step_num, uint8_t step_pin, uint8_t dir_pin)
{
    uint8_t i = step_num-1;
    steppers[i].step_pin = step_pin;
    steppers[i].dir_pin = dir_pin;
    steppers[i].step_count = 0;
    steppers[i].step_direction = 1;
    steppers[i].total_steps = 0;
    steppers[i].min_interval = 1500;
    steppers[i].acceleration = 100;
    

    stepper_gpio_conf.intr_type = GPIO_INTR_DISABLE;
    stepper_gpio_conf.mode = GPIO_MODE_OUTPUT;
    // configures all step and dir pins
    stepper_gpio_conf.pin_bit_mask |= (1ULL<<steppers[i].step_pin) | (1ULL<<steppers[i].dir_pin);
    stepper_gpio_conf.pull_down_en = 0;
    stepper_gpio_conf.pull_up_en = 0;

    gpio_config(&stepper_gpio_conf);
    NUM_STEPPERS++;
}