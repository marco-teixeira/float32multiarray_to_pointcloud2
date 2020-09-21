#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



ros::Publisher pub;
std::string input_topic, output_topic, frame_id;

int height, width;
float near_clip, far_clip, view_angle;





void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
        std::vector<float> depth_raw = msg->data;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	unsigned int datalen = height * width;
        float scale = (far_clip - near_clip) / 1.0;
        std::vector<float> x_scale, y_scale;
        float f = float(float(std::max(height, width)) / 2) / float(tan(view_angle  / 2)); 
        for (int j = 0; j < height; j++)
        {
            float y = (j - height / 2.0);
            for (int i = 0; i < width; i++)
            {
                int k = j * width + i;
                float x = -(i - width / 2.0);
                x_scale.push_back(float(x / f));
                y_scale.push_back(float(y / f));

                float depth = near_clip + scale * depth_raw[k];
                float xyz[3] = {depth * x_scale[k], depth * y_scale[k], depth};
                pcl::PointXYZRGB p;
                p.x = xyz[0];
                p.y = xyz[1];
                p.z = xyz[2];
                p.r = 0;
                p.g = 0;
                p.b = 0;
                cloud->points.push_back(p);
            }
        }
        //cloud->width = cloud->points.size();
        //cloud->height = 1;
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud.get(),output );
        output.header.frame_id = frame_id;
      
        pub.publish (output);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  //Get parameters
  ros::param::param<std::string>("/float32multiarray_to_pointcloud2/input_topic", input_topic, "Float32MultiArray_in");
  ros::param::param<std::string>("/float32multiarray_to_pointcloud2/output_topic", output_topic, "cloud_out");
  
  ros::param::param<std::string>("/float32multiarray_to_pointcloud2/frame_id", frame_id, "frame_id");
  
  ros::param::param<float>("/float32multiarray_to_pointcloud2/near_clip", near_clip, 0.02);
  ros::param::param<float>("/float32multiarray_to_pointcloud2/far_clip", far_clip, 3.5);
  ros::param::param<float>("/float32multiarray_to_pointcloud2/view_angle", view_angle, 57);
  ros::param::param<int>("/float32multiarray_to_pointcloud2/height", height, 480);
  ros::param::param<int>("/float32multiarray_to_pointcloud2/width", width, 640);
 
  
  ros::Subscriber sub = n.subscribe(input_topic, 1000, chatterCallback);
  
  pub = n.advertise<sensor_msgs::PointCloud2> (output_topic, 1);
  ros::spin();
  return 0;
}
