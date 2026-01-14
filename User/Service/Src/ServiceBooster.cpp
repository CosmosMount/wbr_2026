//
// Created by cosmosmount on 2025/9/8.
//

#include "main.h"
#include "tx_api.h"
#include "led.hpp"
#include <cstdint>

TX_THREAD alive_thread;
uint8_t alive_thread_stack[512];

extern TX_THREAD RemoterThread;
extern TX_SEMAPHORE RemoterGot;
extern uint8_t RemoterThreadStack[1024];
extern void RemoterThreadFun(ULONG initial_input);

extern TX_THREAD IMUThread;
extern TX_SEMAPHORE IMUThreadSem;
extern uint8_t IMUThreadStack[4096];
extern void IMUThreadFun(ULONG initial_input);

extern TX_THREAD IMUTempThread;
extern uint8_t IMUTempThreadStack[1024];
extern void IMUTempThreadFun(ULONG initial_input);

extern TX_THREAD RefereeThread;
extern TX_SEMAPHORE RefereeThreadSem;
extern uint8_t RefereeThreadStack[2048];
extern void RefereeThreadFun(ULONG initial_input);

/* Task Semaphores for Alivecheck */
extern TX_SEMAPHORE FunctionThreadSem;
extern TX_SEMAPHORE SolverThreadSem;
extern TX_SEMAPHORE PendulumThreadSem;
extern TX_SEMAPHORE UIThreadSem;

/* EKF Pool */
TX_BYTE_POOL KFPool;
UCHAR KF_PoolBuf[4096] = {0};

/* OneMessage Pool */
TX_BYTE_POOL MsgPool;
UCHAR Msg_PoolBuf[4096] = {0};

[[noreturn]] void alive_thread_entry(ULONG thread_input)
{
    LED_ALL_ON();
    /* Enter into a forever loop. */
    while(1)
    {
        /* Increment thread counter. */
        bool imu_alive = tx_semaphore_get(&IMUThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool referee_alive = tx_semaphore_get(&RefereeThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool function_alive = tx_semaphore_get(&FunctionThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool solver_alive = tx_semaphore_get(&SolverThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool pendulum_alive = tx_semaphore_get(&PendulumThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool ui_alive = tx_semaphore_get(&UIThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        if (imu_alive)
        {
            if (solver_alive && function_alive && referee_alive && ui_alive)
            {
                if (pendulum_alive)
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
        tx_thread_sleep(500);
    }
}

#define TX_NAME(s) const_cast<CHAR*>(s)

extern "C" void ServiceBooster()
{
    /* Create Memory Pools */
    tx_byte_pool_create(
            &KFPool,
            (CHAR *) "KF_Pool",
            KF_PoolBuf,
            sizeof(KF_PoolBuf));

    tx_byte_pool_create(
            &MsgPool,
            (CHAR *) "Msg_Pool",
            Msg_PoolBuf,
            sizeof(Msg_PoolBuf));

    /* Create Threads */
    tx_thread_create(&alive_thread, TX_NAME("alive_thread"),
        alive_thread_entry, 0x1234, alive_thread_stack, sizeof(alive_thread_stack),
        10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&RemoterThread, TX_NAME("RemoterThread"),
        RemoterThreadFun, 0x1234, RemoterThreadStack, sizeof(RemoterThreadStack),
        2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&IMUThread, TX_NAME("IMUThread"),
        IMUThreadFun, 0x1234, IMUThreadStack, sizeof(IMUThreadStack),
        3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&IMUTempThread, TX_NAME("IMUTempThread"),
        IMUTempThreadFun, 0x1234, IMUTempThreadStack, sizeof(IMUTempThreadStack),
        4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&RefereeThread, TX_NAME("RefereeThread"),
        RefereeThreadFun, 0x1234, RefereeThreadStack, sizeof(RefereeThreadStack),
        8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);

    /* Create Semaphore */
    tx_semaphore_create(&RemoterGot, TX_NAME("RemoterGot"), 0);
    tx_semaphore_create(&IMUThreadSem, TX_NAME("IMUThreadSem"), 0);
    tx_semaphore_create(&RefereeThreadSem, TX_NAME("RefereeThreadSem"), 0);
}