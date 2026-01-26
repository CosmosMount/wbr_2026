#include "DMMotor.hpp"
#include "led.hpp"
#include "tx_api.h"
#include "DMMotorHandler.hpp"

TX_THREAD AliveThread;
uint8_t AliveThreadStack[512] = {0};

/* Semaphores for Alivecheck */
extern TX_SEMAPHORE IMUThreadSem;
extern TX_SEMAPHORE RefereeThreadSem;
extern TX_SEMAPHORE FunctionThreadSem;
extern TX_SEMAPHORE PendulumThreadSem;
extern TX_SEMAPHORE UIThreadSem;

[[noreturn]] void AliveThreadFun(ULONG thread_input)
{
    LED_ALL_ON();

    while(1)
    {
        bool imu_alive = tx_semaphore_get(&IMUThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool referee_alive = tx_semaphore_get(&RefereeThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool function_alive = tx_semaphore_get(&FunctionThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool pendulum_alive = tx_semaphore_get(&PendulumThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        if (imu_alive)
        {
            if (function_alive && referee_alive && pendulum_alive)
            {
                if (DMMotorHandler::Instance()->AllMotorAlive())
                {
                    LED_blink(LED_COLOR::LED_GREEN);
                }
                else
                {
                    LED_blink(LED_COLOR::LED_BLUE);
                }
            }
        }
        else 
        {
            LED_blink(LED_COLOR::LED_RED);
        }
        tx_thread_sleep(2);
    }
}