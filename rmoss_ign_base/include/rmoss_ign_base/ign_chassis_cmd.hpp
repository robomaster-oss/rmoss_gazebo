/*******************************************************************************
 *  Copyright (c) 2021 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#ifndef RMOSS_IGN_BASE_IGN_CHASSIS_CMD_PUBLISHER_H
#define RMOSS_IGN_BASE_IGN_CHASSIS_CMD_PUBLISHER_H

#include <ignition/transport/Node.hh>

namespace rmoss_ign_base {

class IgnChassisCmd {
public:
    IgnChassisCmd(const std::shared_ptr<ignition::transport::Node> &ign_node,
               const std::string &ign_chassis_cmd_topic);
    ~IgnChassisCmd() {};

public:
    void publish(double v_x,double v_y,double v_w);
private:
    std::shared_ptr<ignition::transport::Node> ign_node_;
    std::unique_ptr<ignition::transport::Node::Publisher> ign_chassis_cmd_pub_;
};

}

#endif //RMOSS_IGN_BASE_IGN_CHASSIS_CMD_PUBLISHER_H