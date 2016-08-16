#include "map_optimizer.h"

#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <time.h>


#include "ros/ros.h"
#include "ros/console.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/PointCloud.h>


#include <Eigen/Core>

#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH


MapOptimizer::MapOptimizer()
:totle_size_(0), node_(NULL)
{
	init();
}

MapOptimizer::~MapOptimizer()
{
	//free node_ buffer;
	delete node_;
}

void MapOptimizer::init()
{
	if(!private_nh_.getParam(ros::this_node::getName() + "/mapdata_folder", mapdata_folder_))
		mapdata_folder_ = "";
	if(!private_nh_.getParam("baseFrame", baseFrame_))
		baseFrame_ = "map";


	poses_pub = private_nh_.advertise<visualization_msgs::MarkerArray>("/map_optimizer/trajectory", 1, true); //publish latched trajectory
	selected_scans_pub = private_nh_.advertise<sensor_msgs::PointCloud>("/map_optimizer/select_scans", 1, true);
	odom_select_sub = private_nh_.subscribe<map_optimizer_msg::OdomSelectList>("/taimi/odom_selected", 1, [this] (const map_optimizer_msg::OdomSelectList::ConstPtr& msg) { handleOdomSelect(msg); });

	loadMapData();
	publishPoses();
	//std::vector<int> ids = {0, 1, 2, 3};
	//showLaserScans(ids);
}

void MapOptimizer::handleOdomSelect(const map_optimizer_msg::OdomSelectList::ConstPtr& msg)
{
	std::vector<int> ids;
	for(int i = 0; i < msg->id_list.size(); ++i)
	{
		ids.push_back(std::stoi(msg->id_list[i]));
	}
	showLaserScans(ids);
}

void MapOptimizer::loadMapData()
{
	if(mapdata_folder_.empty())
	{
		ROS_INFO("Empty map data folder!");
		return;
	}
	std::string pose_file(mapdata_folder_ + "/pose.csv");

	ROS_INFO("MapOptimizer::loadMapData: %s", pose_file.c_str());
	std::ifstream pose_stream(pose_file.c_str());

	if (!pose_stream.good()) {
		ROS_ERROR("Fail to load pose file: %s", pose_file.c_str());
	    return;
	  }

	int num_poses = 0;
	//pose_stream >> num_poses;
	scans_buf_.clear();
	//scan_ids.reserve(num_poses);
	TNode* parent = NULL;
	std::string line;
	totle_size_ = 0;
	while(std::getline(pose_stream, line))
	//for(int i = 0; i < num_poses; ++i)
	{
		int id;
		double bag_time;
		double weight, gweight;
		GMapping::OrientedPoint pose;

		std::stringstream stream(line);
		std::string s;
		std::getline(stream, s, ',');
		id = std::stoi(s);
		std::getline(stream, s, ',');
		bag_time = std::stof(s);

		std::getline(stream, s, ',');
		weight = std::stof(s);

		std::getline(stream, s, ',');
		gweight = std::stof(s);

		std::getline(stream, s, ',');
		pose.x = std::stof(s);

		std::getline(stream, s, ',');
		pose.y = std::stof(s);

		std::getline(stream, s, ',');
		pose.theta = std::stof(s);

		//pose_stream >> id >> bag_time >> weight >> gweight >> pose.x >> pose.y >> pose.theta;
		ROS_INFO_STREAM("data: "<< id <<", " << bag_time<<", " << weight <<", " << gweight <<", [" << pose.x <<", " << pose.y <<", " << pose.theta<<"]");
		TNode* node = new TNode(pose, weight, parent);
		//load scan for this node
		std::string scan_file(mapdata_folder_ + "/" + std::to_string(id) + ".scan");
		auto scanPtr = loadLaserScan(scan_file);
		scans_buf_[id] = scanPtr;
		node_map_[id] = node;
		totle_size_++;
		parent = node;//TODO: reverse the list.
	}
	//reserve the node list
//	auto p0 = parent, p1 = parent;
//	for(GMapping::GridSlamProcessor::TNode* n = parent;  n;  n = n->parent)
//	{
//		if(n==parent) continue;
//		if(p0 == parent) p0->parent = 0;
//		else p0->parent = p1;
//
//		p1 = p0;
//		p0 = n;
//	}
//	parent = p0;

	if(node_) delete node_;
	node_ = parent;
	pose_stream.close();
}

sensor_msgs::LaserScanPtr MapOptimizer::loadLaserScan(const std::string& scan_file)
{

	std::ifstream scan_stream(scan_file.c_str());
	if (!scan_stream.good()) {
		ROS_ERROR("Fail to load laser scan file: %s", scan_file.c_str());
		return 0;
	}
	auto scan = boost::make_shared<sensor_msgs::LaserScan>();
	std::string line;
	//read out laser info
	std::getline(scan_stream, line);
	std::stringstream stream(line);
	std::string s;
	std::getline(stream, s, ',');
	scan->angle_min = std::stof(s);
	std::getline(stream, s, ',');
	scan->angle_max = std::stof(s);
	std::getline(stream, s, ',');
	scan->angle_increment = std::stof(s);
	std::getline(stream, s, ',');
	scan->time_increment = std::stof(s);
	std::getline(stream, s, ',');
	scan->scan_time = std::stof(s);
	std::getline(stream, s, ',');
	scan->range_min = std::stof(s);
	std::getline(stream, s, ',');
	scan->range_max = std::stof(s);

	//read out number laser scan beams
	std::getline(scan_stream, line);
	int n = std::stoi(line);

	//read out laser scans beams
	scan->ranges.reserve(n);
	std::getline(scan_stream, line);
	std::stringstream stream_beams(line);
	for(int i = 0; i < n; ++i)
	{
		std::string s;
		std::getline(stream_beams, s, ',');
		scan->ranges.emplace_back();
		auto& beam = scan->ranges.back();
		beam = std::stof(s);
	}

//	ROS_INFO_STREAM("laser scan loaded: "<<*scan);
	return scan;
}

void MapOptimizer::showLaserScans(const std::vector< int >& ids)
{
	sensor_msgs::PointCloudPtr msg_out = boost::make_shared< sensor_msgs::PointCloud >();
	msg_out->header.stamp = ros::Time::now();
	msg_out->header.frame_id = baseFrame_;
	for(int i = 0; i < ids.size(); ++i)
	{
		if(node_map_.find(i) == node_map_.end())
			continue;
		auto& n = node_map_[ids[i]];
		auto A = tf::Transform(tf::createQuaternionFromRPY( 0, 0, n->pose.theta ), tf::Point(n->pose.x, n->pose.y, 0 ));
		auto& scan_ = scans_buf_[ids[i]];
		sensor_msgs::PointCloudPtr pc = boost::make_shared< sensor_msgs::PointCloud >();
		ROS_INFO_STREAM("scan size: "<<scan_->ranges.size());
		projector_.projectLaser(*scan_, *pc);
		for(int j = 0; j < pc->points.size(); ++j)
		{
			tf::Vector3 tmp(pc->points[j].x, pc->points[j].y, pc->points[j].z);
			auto v = A(tmp);

			msg_out->points.emplace_back();
			auto& p_out = msg_out->points.back();
			p_out.x = float(v.x()); p_out.y = float(v.y()); p_out.z = float(v.z());
//			ROS_INFO("point[%f, %f]", p_out.x, p_out.y);
		}
	}
	ROS_INFO_STREAM("Final size: " << msg_out->points.size());
	selected_scans_pub.publish(msg_out);

}

void MapOptimizer::publishPoses()
{
	ROS_INFO("%s: create poses marker array", __FUNCTION__);
	visualization_msgs::MarkerArray posesMsg;
	ros::Time time = ros::Time::now();
	int id = totle_size_;
	boost::optional<Eigen::Vector2f> last_position;
	for(GMapping::GridSlamProcessor::TNode* n = node_;
	      n;
	      n = n->parent)
	  {
	    ROS_INFO("  %.3f %.3f %.3f",
	              n->pose.x,
	              n->pose.y,
	              n->pose.theta);
	    /*if(!n->reading)
	    {
	      ROS_DEBUG("Reading is NULL");
	      continue;
	    }*/
	    auto v = Eigen::Vector2f(n->pose.x, n->pose.y);
	    double dist = 0.5;
	    if(last_position)
	    	dist = (v - *last_position).norm();

	    last_position = v;

	    visualization_msgs::Marker marker;
	    marker.header.frame_id = baseFrame_;
		marker.header.stamp = time;
		marker.ns = "map_optimizer";
		marker.id = --id;
		ROS_INFO_STREAM("id: " << marker.id);
		//marker.id = std::string("pos_") + itoa(id);
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = dist;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
		marker.pose.position.x = n->pose.x;
		marker.pose.position.y = n->pose.y;
		marker.pose.position.z = 0.0;
		tf::Quaternion q = tf::createQuaternionFromRPY( 0, 0, n->pose.theta);
		marker.pose.orientation.x = q.x();
		marker.pose.orientation.y = q.y();
		marker.pose.orientation.z = q.z();
		marker.pose.orientation.w = q.w();
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 0.5;
		marker.lifetime = ros::Duration();
		posesMsg.markers.push_back(marker);
	  }
	ROS_INFO_STREAM("posesMsg.markers size: " << posesMsg.markers.size());
	poses_pub.publish(posesMsg);

}
