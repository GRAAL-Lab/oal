#ifndef OAL_DEFINES_HPP
#define OAL_DEFINES_HPP
#include <string>

namespace oal {

namespace EncounterTypes {
    inline const std::string HEAD_ON = "HEAD_ON";
    inline const std::string VH_OVERTAKING = "VH_OVERTAKING";
    inline const std::string VH_CROSSING_FROM_LEFT = "VH_CROSSING_FROM_LEFT";
    inline const std::string VH_CROSSING_FROM_RIGHT = "VH_CROSSING_FROM_RIGHT";
    inline const double HeadOnAngle = (15 * (M_PI / 180));
    inline const double OvertakingAngle = (112 * (M_PI / 180));
}

namespace NodeSearch {
    namespace DiscoverySource {
        inline const std::string START = "START"; // First node
        inline const std::string EXIT_VX = "EXIT_VX"; // Starting inside a bb
        inline const std::string AIM_TO_GOAL = "AIM_TO_GOAL"; // First exploration phase is toward the goal
        inline const std::string EXPLORATION = "EXPLORATION"; // Looking to reach other obstacles/vxs
        inline const std::string CLOSEST_TO_GOAL = "CLOSEST_TO_GOAL"; // Goal is in bb
    };
    namespace DiscardReason {
        inline const std::string PREVIOUSLY_DISCARDED = "PREVIOUSLY_DISCARDED";
        inline const std::string NON_COMPLIANT = "NON_COMPLIANT";
        inline const std::string COLLISION = "COLLISION";
        inline const std::string EXPLORED = "EXPLORED";
    };
}

namespace SearchResult {
    inline const std::string FOUND = "FOUND";
    inline const std::string PARTIAL = "PARTIAL";
    inline const std::string FAIL = "FAIL";
};

}
#endif // OAL_DEFINES_HPP