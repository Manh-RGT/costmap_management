#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/inflation_layer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/shared_ptr.hpp>

class CostmapModifier {
public:
    CostmapModifier() : tf_listener_(tf_buffer_) {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle();

        // Subscribe to the local costmap topic
        costmap_sub_ = nh_.subscribe("/sirbot1/Depth_layer", 1, &CostmapModifier::costmapCallback, this);

        // Publisher for the modified costmap
        costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("modified_costmap", 1);

        // Initialize Costmap2DROS
        costmap_2d_ros_ = boost::make_shared<costmap_2d::Costmap2DROS>("costmap_2d_ros", tf_buffer_);

        // Add inflation layer
        inflation_layer_.reset(new costmap_2d::InflationLayer());
        inflation_layer_->initialize(costmap_2d_ros_->getLayeredCostmap(), costmap_2d_ros_->getName() + "/inflation", &tf_buffer_);

        // Configure inflation layer parameters (you can adjust these)
        nh_.param("inflation_radius", inflation_radius_, 0.5); // Default: 0.5 meters
        nh_.param("cost_scaling_factor", cost_scaling_factor_, 2.58); // Default scaling factor
        inflation_layer_->setInflationParameters(inflation_radius_, cost_scaling_factor_);

        // Add the inflation layer to the costmap
        costmap_2d_ros_->getLayeredCostmap()->addPlugin(inflation_layer_);

        // Start the costmap
        costmap_2d_ros_->start();

        ROS_INFO("CostmapModifier node initialized.");
    }

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        // Update the costmap with the incoming data
        if (costmap_2d_ros_->getLayeredCostmap()->isCurrent()) {
            costmap_2d_ros_->updateMap();
        } else {
            ROS_WARN("Costmap is not current, skipping update.");
            return;
        }

        // Get the underlying Costmap2D object
        costmap_2d::Costmap2D* costmap = costmap_2d_ros_->getCostmap();

        // Resize the Costmap2D to match the incoming message dimensions
        costmap->resizeMap(msg->info.width, msg->info.height, msg->info.resolution,
                        msg->info.origin.position.x, msg->info.origin.position.y);

        // Create a new OccupancyGrid message to publish
        nav_msgs::OccupancyGrid modified_costmap;
        modified_costmap.header = msg->header;
        modified_costmap.info = msg->info;

        // Copy the costmap data into the message
        modified_costmap.data.resize(costmap->getSizeInCellsX() * costmap->getSizeInCellsY());
        for (unsigned int y = 0; y < costmap->getSizeInCellsY(); y++) {
            for (unsigned int x = 0; x < costmap->getSizeInCellsX(); x++) {
                unsigned char cost = costmap->getCost(x, y);
                modified_costmap.data[y * costmap->getSizeInCellsX() + x] = cost;
            }
        }

        // Publish the modified costmap
        costmap_pub_.publish(modified_costmap);
    }
private:
    ros::NodeHandle nh_;
    ros::Subscriber costmap_sub_;
    ros::Publisher costmap_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_2d_ros_;
    boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer_;
    double inflation_radius_;
    double cost_scaling_factor_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "costmap_modifier_node");
    CostmapModifier node;
    ros::spin();
    return 0;
}