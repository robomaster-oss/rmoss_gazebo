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
#ifndef RMOSS_IGN_BASE_PID_H
#define RMOSS_IGN_BASE_PID_H

#include <ignition/math/PID.hh>

namespace rmoss_ign_base {

struct PidParam{
    double p=1;
    double i=0;
    double d=0;
    double imax=1;
    double imin=-1;
    double cmdmax=100;
    double cmdmin=-100;
    double offset=0;
};

}

#endif //RMOSS_IGN_BASE_PID_H