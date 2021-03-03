// A mapping node for educational purposes

// roscpp
#include "ros/ros.h"

// Messages and services that we need
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

// someone was lazy here...
#define SWAP(a, b)  {a ^= b; b ^= a; a ^= b;}

using namespace std;

class MappingNode {

public:
	MappingNode(ros::NodeHandle n);
	~MappingNode();

private:
	ros::NodeHandle nodeHandle;

	// we use the OccupancyGrid structure representing the reflection map
	nav_msgs::OccupancyGrid map;

	// subscribe to laser, use a msg_filter to ensure that we can associate an odometry pose with the scan
	tf::TransformListener tfListener;
	message_filters::Subscriber<sensor_msgs::LaserScan> laserScanSubscriber;
	tf::MessageFilter<sensor_msgs::LaserScan> laserScanFilter;

	// in order to visualize the map, we have to publish it
	ros::Publisher mapPublisher;
	ros::ServiceServer mapService;

	// Message and service callbacks
	void laserReceived( const sensor_msgs::LaserScanConstPtr& laser_scan );
	bool mapRequested( nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res );

	///////////////////////////////////////////////////
	// <helpful methods>

	// implementation of the bresenham algorithm: http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    //Bresenham's line algorithm is a line drawing algorithm that determines the points of an n-dimensional
    // raster that should be selected in order to form a close approximation to a straight line between two points.
	//  returns all points from the starting point (x0, y0) and end point (x1, y1)
	//  starting and end point are included in the returned result.
	vector<pair<int, int> > bresenhamLine( int x0, int y0, int x1, int y1 );
	
	// use this method to update the reflection map
	void updateReflectionMapValue(int x, int y, double value);

    
	// </helpful methods>
	///////////////////////////////////////////////////

    
    
    
	///////////////////////////////////////////////////
	// <your declarations>
    vector<int > hit_count;
     
    vector<int > miss_count;

    //float miss_count[1000000];
    //float hit_count[1000000];
	///////////////////////////////////////////////////
};

MappingNode::MappingNode(ros::NodeHandle n) :
	nodeHandle(n),
	laserScanSubscriber(nodeHandle, "scan", 5),
	laserScanFilter(laserScanSubscriber, tfListener, "odom", 5)
{
	// initialize map meta data
	// map is 20 x 20 meters, resolution is 0.02 meters per grid cell
	this->map.info.resolution = 0.02;
	this->map.info.width = 1000;
	this->map.info.height = 1000;
	this->map.info.origin.position.x = -this->map.info.resolution * this->map.info.width / 2.0;
	this->map.info.origin.position.y = -this->map.info.resolution * this->map.info.height / 2.0;
	this->map.info.origin.position.z = 0.0;
	this->map.info.origin.orientation.x = 0.0;
	this->map.info.origin.orientation.y = 0.0;
	this->map.info.origin.orientation.z = 0.0;
	this->map.info.origin.orientation.w = 1.0;
	this->map.data.resize( this->map.info.width * this->map.info.height );

	// initialize map
	for (unsigned int i = 0; i < this->map.info.width; i++) {
		for (unsigned int j = 0; j < this->map.info.height; j++) {
			int index = j*this->map.info.height + i;
			this->map.data[index] = -1; // -1 means unknown
		}
	}
	///////////////////////////////////////////////////
	// <your code>

	// YOU may initialize all the data structures you need
	// to implement simple counting method here!

	
	// </your code>
    hit_count.resize(this->map.data.size());
    miss_count.resize(this->map.data.size());
   // float miss_count[1000000]={};
   // float hit_count[1000000]={};
	///////////////////////////////////////////////////

	// subscribe to laser scan
	laserScanFilter.registerCallback( boost::bind( &MappingNode::laserReceived, this, _1 ) );
//	laserScanFilter.setTolerance(ros::Duration(0.01));

	// publish the map we're building
	this->mapPublisher = this->nodeHandle.advertise<nav_msgs::OccupancyGrid>( "map", 2 );
	this->mapService = this->nodeHandle.advertiseService( "map", &MappingNode::mapRequested, this );

	// publish the unknown map for the first time
	this->mapPublisher.publish( this->map );
}

MappingNode::~MappingNode() {
}


void MappingNode::laserReceived( const sensor_msgs::LaserScanConstPtr& laserScan ) {
	// what's the robot's odometry when this scan was taken?
	tf::Stamped<tf::Pose> ident( tf::Transform( tf::createIdentityQuaternion(), tf::Vector3(0,0,0) ), laserScan->header.stamp, "base_laser" );
	tf::Stamped<tf::Pose> odomPose;
	try {
		this->tfListener.transformPose( "odom", ident, odomPose );
	}
	catch( tf::TransformException e ) {
		ROS_WARN( "Failed to compute odom pose, skipping scan (%s)", e.what() );
		return;
	}

    
    
    
    // transform pose of robot to world coordinates
    int robotX = odomPose.getOrigin().x() / this->map.info.resolution + this->map.info.width / 2;
    int robotY = odomPose.getOrigin().y() / this->map.info.resolution + this->map.info.height / 2;
    double robotTheta = tf::getYaw( odomPose.getRotation() );

    ///////////////////////////////////////////////////
	// <your code>
    //for(std::vector<int>::size_type i = 0; i != v.size(); i++)
    for (int i = 0; i <laserScan->ranges.size(); i++)
    {
    //the angle of the laser in map coordinate
        float realTheta=robotTheta+laserScan->angle_min+i*laserScan->angle_increment;
    //range data [m] (Note: values < range_min or > range_max should be discarded)
        
        //from angle to coordinate
        //in world coordinate
        if (laserScan->ranges[i]>=laserScan->range_min && laserScan->ranges[i]<=laserScan->range_max)
        {
            // in real world, the unit of ranges is [m]
            float X_w=laserScan->ranges[i]*cos(realTheta)+odomPose.getOrigin().x();
            float Y_w=laserScan->ranges[i]*sin(realTheta)+odomPose.getOrigin().y();
             
            //in map coordinate
             int X_map=X_w / this->map.info.resolution + this->map.info.width / 2;
             int Y_map=Y_w / this->map.info.resolution + this->map.info.height / 2;
            // get the vectors come from x0,y0 to x1,y1
            vector<pair<int, int> > vec_via= MappingNode::bresenhamLine(robotX, robotY ,X_map, Y_map);
            
            for (int j = 0; j<vec_via.size(); ++j)
            {
                //double prob;
                //this could be a problem
                pair<int, int> point_via=vec_via[j];
                int index = point_via.first + point_via.second*this->map.info.width;
                if (j<vec_via.size()-1)
                {
                    miss_count[index]=miss_count[index]+1;
                    }
                else
                {
                    hit_count[index]=hit_count[index]+1;
                }
                double prob=hit_count[index]/static_cast<double>(miss_count[index]+hit_count[index]);
                updateReflectionMapValue(point_via.first,point_via.second,prob);
                
                    
            }
    
        }
    }

    
    
	// compute the reflection grid update value here
	//  you should use updateReflectionMapValue(int x, int y, double value) for assigning the value
	//  that you computed
	
	// 	the method bresenhamLine( int x0, int y0, int x1, int y1 ) will also be helpful.
	
	// odomPose contains the *current* pose of the robot, estimated from past odometry data.
    // You can use robotX, robotY, and robotTheta for the robot position and rotation on the map.
	
	// </your code>

	// publish result
	this->map.header.stamp = ros::Time::now();
	this->map.header.frame_id = "/odom";
	this->mapPublisher.publish( this->map );
}

void MappingNode::updateReflectionMapValue(int i, int j, double value) {
	int index = j*this->map.info.width + i;
	this->map.data[index] = 100*value; // *100 is just for better visibility in visualization
}

vector<pair<int, int> > MappingNode::bresenhamLine( int x0, int y0, int x1, int y1 ) {
	vector<pair<int, int> > result;

	bool steep = abs(y1 - y0) > abs(x1 - x0);

	if( steep ) {
		SWAP( x0, y0 );
		SWAP( x1, y1 );
	}
	if( x0 > x1 ) {
		SWAP( x0, x1 );
		SWAP( y0, y1 );
	}

	int deltax = x1 - x0;
	int deltay = abs(y1 - y0);
	int error = deltax / 2;
	int y = y0;
	int ystep = ( y0 < y1 ) ? 1 : -1;

	for( int x = x0; x <= x1; x++ ) {
		if ( steep ) {
			// cell (y,x) is crossed
			result.push_back(pair<int, int>(y, x));
		} else {
			// cell (x,y) is crossed
			result.push_back(pair<int, int>(x, y));
		}

		error = error - deltay;
		if( error < 0 ) {
			y += ystep;
			error += deltax;
		}
	}
	
	return result;
}

bool MappingNode::mapRequested( nav_msgs::GetMap::Request &request, nav_msgs::GetMap::Response &response ) {
	if( this->map.info.width && this->map.info.height )	{
		this->map.header.stamp = ros::Time::now();
		response.map = this->map;
		return true;
	}
	else
		return false;
}


int main( int argc, char** argv ) {
	ros::init( argc, argv, "mapper" );
	ros::NodeHandle nh;

	MappingNode mn( nh );

	ros::spin();

	return 0;
}
