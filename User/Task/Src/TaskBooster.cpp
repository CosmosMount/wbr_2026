#include "main.h"
#include "tx_api.h"

extern TX_THREAD PendulumThread;
extern uint8_t PendulumThreadStack[4096];
extern void PendulumThreadFun(ULONG initial_input);

extern TX_THREAD SolverThread;
extern uint8_t SolverThreadStack[4096];
extern void SolverThreadFun(ULONG initial_input);

extern TX_THREAD FunctionThread;
extern uint8_t FunctionThreadStack[2048];
extern void FunctionThreadFun(ULONG initial_input);

extern TX_THREAD UIThread;
extern uint8_t UIThreadStack[2048];
extern void UIThreadFun(ULONG initial_input);

extern TX_SEMAPHORE TOFGot;

#define TX_NAME(s) const_cast<CHAR*>(s)
extern "C" void TaskBooster(void)
{

    tx_thread_create(&PendulumThread, TX_NAME("PendulumThread"), PendulumThreadFun, 0x1234,
                     PendulumThreadStack, sizeof(PendulumThreadStack),
                     6, 6, TX_NO_TIME_SLICE, TX_AUTO_START);
    
    tx_thread_create(&SolverThread, TX_NAME("SolverThread"), SolverThreadFun, 0x1234,
                     SolverThreadStack, sizeof(SolverThreadStack),
                     5, 5, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&FunctionThread, TX_NAME("FunctionThread"), FunctionThreadFun, 0x1234,
                     FunctionThreadStack, sizeof(FunctionThreadStack),
                     7, 7, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&UIThread, TX_NAME("UIThread"), UIThreadFun, 0x1234,
                     UIThreadStack, sizeof(UIThreadStack),
                     8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_semaphore_create(&TOFGot, TX_NAME("TOFGot"), 0);
}