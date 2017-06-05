#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
// PCL specific includes
 #include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/filters/voxel_grid.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#define  PI M_PI
ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudScans[16];
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
int p_obstacles_detection_scans_;//选多少条
int p_obstacles_detection_interval_;//间隔多少条
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2)
{

	 pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn);
	 std::vector<int> indices;
     pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
     int cloudSize = laserCloudIn->points.size();

     float startOri = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);//起始角度，前面取负号是因为激光雷达是从顶上看顺时钟扫描
     float endOri = -atan2(laserCloudIn->points[cloudSize - 1].y,
                         laserCloudIn->points[cloudSize - 1].x) + 2 * PI;//结束角度
//ROS_INFO ("laserCloudIn->points[cloudSize - 33].x: %f  , laserCloudIn->points[cloudSize - 33].y: %f ", laserCloudIn->points[cloudSize - 33].x,laserCloudIn->points[cloudSize - 33].y);
 //ROS_ERROR ("startOri: %f  , endOri: %f ", startOri,endOri);
   if (endOri - startOri > 3 * PI) {
     endOri -= 2 * PI;
   } else if (endOri - startOri < PI) {
     endOri += 2 * PI;
   }
 //ROS_ERROR ("XstartOri: %f  , XendOri: %f ", startOri,endOri);
   bool halfPassed = false;
   pcl::PointXYZI point;
   for (int i = 0; i < cloudSize; i++) {
     point.x = laserCloudIn->points[i].x;
     point.y = laserCloudIn->points[i].y;
     point.z = laserCloudIn->points[i].z;

     float angle = atan(point.z / sqrt(point.y * point.y + point.x * point.x)) * 180 / PI;//计算每点在俯仰向的角度

    /* int scanID = 0;//这是按传感器定义的ID存的
     if (angle>0)
     {
       scanID = int(angle + 0.5);
     }
     else
     { 
       scanID = int(angle + 15+0.5);

     }*/
     int scanID = int ((angle+16)*0.5);//这是按角度顺序存的
         if(scanID<0){
             scanID=0;
         }
         if(scanID>15){
             scanID=15;
         }

     float ori = -atan2(point.y, point.x);//计算每点的方位向角度值
 //ROS_INFO ("scanID: %d angle: %f ori:%f px: %f py: %f pz: %f", scanID,angle,ori,point.x,point.y,point.z);
 //ROS_WARN ("scanID: %d angle: %f ori:%f ", scanID,angle,ori);
     if (!halfPassed) {
       if (ori < startOri - PI / 2) {
         ori += 2 * PI;
       } else if (ori > startOri + PI * 3 / 2) {
         ori -= 2 * PI;
       }
 
       if (ori - startOri > PI) {
         halfPassed = true;
       }
     } else {
       ori += 2 * PI;
 
       if (ori < endOri - PI * 3 / 2) {
         ori += 2 * PI;
       } else if (ori > endOri + PI / 2) {
         ori -= 2 * PI;
       } 
     }
 //ROS_INFO ("XscanID: %d Xangle: %f Xori:%f ", scanID,angle,ori);
    float relTime = (ori - startOri) / (endOri - startOri);//角度比？
    // float relTime = 0 ;
     point.intensity = scanID + 0.1 * relTime;

    
     laserCloudScans[scanID]->push_back(point);
     //laserCloudScans[0]->push_back(point);

   }

  for (int i = 0,j = 0; (i < 16) && (j < p_obstacles_detection_scans_); i=i+p_obstacles_detection_interval_,j=j+1) {
     *laserCloud += *laserCloudScans[i];//只取所需的n线
   }

   sensor_msgs::PointCloud2 laserCloud2;
   pcl::toROSMsg(*laserCloud, laserCloud2);
   laserCloud2.header.stamp = laserCloudIn2->header.stamp;
   laserCloud2.header.frame_id = "/velodyne";
   pub.publish(laserCloud2);


   laserCloudIn->clear();
   laserCloud->clear();
   for (int i = 0; i < 16; i++) {
       laserCloudScans[i]->points.clear();
     }
}

int main (int argc, char** argv)
{
  	// Initialize ROS
  	ros::init (argc, argv, "velodyne_points_filter");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	nh_priv.param("obstacles_detection_scans", p_obstacles_detection_scans_, 4);
	nh_priv.param("obstacles_detection_interval", p_obstacles_detection_interval_, 2);
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/velodyne_points", 2, cloud_cb);
	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_filtered", 2);
	// Spin
	 for (int i = 0; i < 16; i++) {
	     laserCloudScans[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
	   }
	ros::spin ();
}
