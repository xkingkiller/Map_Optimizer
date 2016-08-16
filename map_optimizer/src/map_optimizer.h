#include "ros/ros.h"

#include "gmapping/gridfastslam/gridslamprocessor.h"

//#include "g2o/core/sparse_optimizer.h"
//#include "g2o/types/slam2d/vertex_se2.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include "map_optimizer_msg/OdomSelectList.h"

#include <map>
#include <vector>

class MapOptimizer
{
	//using TNode = GMapping::GridSlamProcessor::TNode;
	typedef GMapping::GridSlamProcessor::TNode TNode;

  public:
	MapOptimizer();
    ~MapOptimizer();

    void init();
  private:
    ros::NodeHandle private_nh_;
    void loadMapData();
    sensor_msgs::LaserScanPtr loadLaserScan(const std::string& scan_file);
    //ros parameters
    std::string mapdata_folder_;
    std::string baseFrame_;
    //mapping trajectory
    TNode* node_;
    std::map< int, sensor_msgs::LaserScanPtr > scans_buf_;
    std::map< int, TNode* > node_map_;
    int totle_size_;
    //
    ros::Publisher poses_pub;
    void publishPoses();
    ros::Publisher selected_scans_pub;
    void showLaserScans(const std::vector< int >& ids);
    //Handle for selection event
    ros::Subscriber odom_select_sub;
    void handleOdomSelect(const map_optimizer_msg::OdomSelectList::ConstPtr& msg);
    //
    laser_geometry::LaserProjection projector_;


};
