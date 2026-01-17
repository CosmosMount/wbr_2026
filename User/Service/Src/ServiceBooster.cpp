#include "main.h"
#include "tx_api.h"

extern TX_THREAD AliveThread;
extern uint8_t AliveThreadStack[512];
extern void AliveThreadFun(ULONG initial_input);

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

/* EKF Pool */
TX_BYTE_POOL KFPool;
UCHAR KF_PoolBuf[4096] = {0};

/* OneMessage Pool */
TX_BYTE_POOL MsgPool;
UCHAR Msg_PoolBuf[4096] = {0};

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

    /* Create Service Threads */
    tx_thread_create(&AliveThread, TX_NAME("AliveThread"),
        AliveThreadFun, 0x1234, AliveThreadStack, sizeof(AliveThreadStack),
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

    /* Create Semaphores */
    tx_semaphore_create(&RemoterGot, TX_NAME("RemoterGot"), 0);
    tx_semaphore_create(&IMUThreadSem, TX_NAME("IMUThreadSem"), 0);
    tx_semaphore_create(&RefereeThreadSem, TX_NAME("RefereeThreadSem"), 0);
}