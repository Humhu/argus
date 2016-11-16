#pragma once

#include "argus_utils/utils/MatrixUtils.h"
#include "fieldtrack/utils/FilterInfo.h"

#include <argus_msgs/FilterStepInfo.h>

namespace argus
{

PredictInfo MsgToPredict( const argus_msgs::FilterStepInfo& msg );
argus_msgs::FilterStepInfo PredictToMsg( const PredictInfo& info );

UpdateInfo MsgToUpdate( const argus_msgs::FilterStepInfo& msg );
argus_msgs::FilterStepInfo UpdateToMsg( const UpdateInfo& info );

}