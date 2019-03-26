#include "SP_Types.h"

#ifndef SP_MAPPING_TYPE_H
#define SP_MAPPING_TYPE_H

/// A struct to store occupancy map meta data.
/// More information can be found at http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html
typedef struct
{
    float resolution;              /*!< Map resolution in [m/cell] */
    uint32_t width;                /*!< width : width of occupancy map. */
    uint32_t height;               /*!< height : height of occupancy map. */
    float offset[3];               /*!< origin[m, m, rad] of the map. Grid traversal starts from this point. */
}
SP_MapMetaData;

///  A struct to store occupancy map data, and its corresponding meta data.
/// More information can be found at http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
typedef struct
{
    SP_MapMetaData mapMetaData;    /*!< stores basic information about the characterists of the OccupancyGrid. */
    int8_t* data;                  /*!< occupancy map data. Occupancy probabilities are in the range [0,100].  Unknown is -1. */
}
SP_OccupancyGridMsg;

#endif // SP_MAPPING_TYPE_H
