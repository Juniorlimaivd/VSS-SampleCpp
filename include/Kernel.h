//
// Created by john on 07/10/18.
//

#ifndef VSS_SAMPLE_KERNEL_H
#define VSS_SAMPLE_KERNEL_H

#include <Communications/CommandSenderAdapter.h>
#include <Communications/DebugSenderAdapter.h>
#include <Communications/StateReceiverAdapter.h>

class Kernel {
public:
    Kernel(vss::ExecutionConfig);

    void run();

private:
    vss::ExecutionConfig executionConfig;

    vss::State state;
    vss::Debug debug;
    vss::Command command;

    ICommandSenderAdapter* commandSender;
    IDebugSenderAdapter* debugSender;
    IStateReceiverAdapter* stateReceiver;

    void sendCommand();
    void sendDebug();
};

#endif //VSS_SAMPLE_KERNEL_H