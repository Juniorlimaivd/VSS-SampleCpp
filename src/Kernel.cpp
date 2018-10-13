//
// Created by john on 07/10/18.
//

#include <Kernel.h>

Kernel::Kernel(vss::ExecutionConfig& executionConfig) {
    this->executionConfig = executionConfig;

    // Constrói o CommandSender com base no EnvironmentType
    commandSenderFactory = new CommandSenderFactory();

    commandSender = commandSenderFactory->create(this->executionConfig);
    debugSender = new DebugSenderAdapter(this->executionConfig);
    stateReceiver = new StateReceiverAdapter(this->executionConfig);

    this->team = new Team(this->executionConfig);
}

void Kernel::run() {
    while(true) {
        this->state = stateReceiver->receive(vss::FieldTransformationType::None);

        this->command = team->getCommandFromState(state);
        this->commandSender->send(command);

        this->sendDebug();
    }
}

void Kernel::sendDebug(){
    this->debug = vss::Debug();

    for(unsigned int i = 0 ; i < 3 ; i++){
        debug.stepPoints.push_back(vss::Point(85 + rand()%20, 65 + rand()%20));
    }

    for(unsigned int i = 0 ; i < 3 ; i++){
        debug.finalPoses.push_back(vss::Pose(85 + rand()%20, 65 + rand()%20, rand()%20));
    }

    for(unsigned int i = 0 ; i < 3 ; i++){
        vss::Path path;
        path.points.push_back(vss::Point(85, 65));
        path.points.push_back(vss::Point(85 + rand()%20, 65 + rand()%20));
    }

    this->debugSender->send(debug);
}


