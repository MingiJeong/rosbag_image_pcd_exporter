#include <string>
#include <fstream>
#include <iostream>
#include <ctime>
#include <sstream>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "ros/ros.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>


// TODO (2023.11.27)
// 1. saving time from time sync function, not separate image callback timer function
// 2. saving altogether of 4 images of ouster or not.
// 3. do we need pointcloud republishing?

class ImageCloudDataExport
{
public:
  ImageCloudDataExport();

  void Run();

private:

  ros::NodeHandle node_handle_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber image_sub_;
  ros::Subscriber ir_image_sub_;
  ros::Subscriber refl_image_sub_;
  ros::Subscriber range_image_sub_;
  ros::Subscriber sig_image_sub_;

  ros::Subscriber cloud_sync_sub_original_;
  ros::Publisher cloud_sync_converter_pub_;
  ros::Time time_stamp_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sync_sub_;
  // message_filters::Subscriber<sensor_msgs::Image> *image_sync_sub_;
  message_filters::Subscriber<sensor_msgs::CompressedImage> *image_sync_sub_;

  typedef
  // message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> SyncPolicyT;
  message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CompressedImage> SyncPolicyT;

  message_filters::Synchronizer<SyncPolicyT>
    *data_synchronizer_;

  size_t image_frame_counter_, cloud_frame_counter_;
  bool sync_topics_;
  bool ouster_use_;
  int leading_zeros;
  std::string path_pointcloud_str_, path_image_str_, path_timestamp_str_;
  std::string path_ir_image_str_, path_refl_image_str_, path_range_image_str_, path_sig_image_str_; 
  cv::Mat ir_image_saver, refl_image_saver, range_image_saver, sig_image_saver;

  // Callback functions 
  void LidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);
  void LidarCloudCallbackTimer(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud_timer); // for republishing

  // void ImageCallback(const sensor_msgs::ImageConstPtr &in_image_msg);
  void ImageCallback(const sensor_msgs::CompressedImageConstPtr &in_image_msg);
  void ImageCallbackTimer(const sensor_msgs::CompressedImageConstPtr &in_image_msg_timer); // for time saving

  void SyncedDataCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud,
    const sensor_msgs::CompressedImageConstPtr &in_image_msg);
  // const sensor_msgs::ImageConstPtr &in_image_msg);

  // ouster other message
  void IrImageCallback(const sensor_msgs::ImageConstPtr &in_ir_image_msg);
  void RefImageCallback(const sensor_msgs::ImageConstPtr &in_refl_image_msg);
  void RangeImageCallback(const sensor_msgs::ImageConstPtr &in_range_image_msg);
  void SigImageCallback(const sensor_msgs::ImageConstPtr &in_sig_image_msg);
  void SaveImageFile(const cv::Mat &to_save_image, const std::string to_save_path);

};

ImageCloudDataExport::ImageCloudDataExport() :
  node_handle_()
{
  image_frame_counter_ = 0;
  cloud_frame_counter_ = 0;
}


// void ImageCloudDataExport::ImageCallback(const sensor_msgs::ImageConstPtr &in_image_msg)
void ImageCloudDataExport::ImageCallback(const sensor_msgs::CompressedImageConstPtr &in_image_msg)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat current_image = cv_image->image;

  std::string string_counter = std::to_string(image_frame_counter_);
  std::string file_path = path_image_str_ + std::string(leading_zeros - string_counter.length(), '0') +
                          string_counter + ".jpg";

  cv::imwrite(file_path, current_image);
  if(!sync_topics_)
  {
    image_frame_counter_++;
  }

}

void ImageCloudDataExport::ImageCallbackTimer(const sensor_msgs::CompressedImageConstPtr &in_image_msg_timer)
{
  // saving image's timestamp
  // (2023.11.27)
  // next time no need to have this separately 
  // it additionally save aside from time sync part -> do this inside the function of time sync
  time_stamp_ = in_image_msg_timer->header.stamp;  
}


void ImageCloudDataExport::IrImageCallback(const sensor_msgs::ImageConstPtr &in_ir_image_msg)
{
  cv_bridge::CvImagePtr cv_ir_image = cv_bridge::toCvCopy(in_ir_image_msg, sensor_msgs::image_encodings::BGR8);
  ir_image_saver = cv_ir_image->image;
}

void ImageCloudDataExport::RefImageCallback(const sensor_msgs::ImageConstPtr &in_refl_image_msg)
{
  cv_bridge::CvImagePtr cv_refl_image = cv_bridge::toCvCopy(in_refl_image_msg, sensor_msgs::image_encodings::BGR8);
  refl_image_saver = cv_refl_image->image;
}

void ImageCloudDataExport::RangeImageCallback(const sensor_msgs::ImageConstPtr &in_range_image_msg)
{
  cv_bridge::CvImagePtr cv_range_image = cv_bridge::toCvCopy(in_range_image_msg, sensor_msgs::image_encodings::BGR8);
  range_image_saver = cv_range_image->image;
}

void ImageCloudDataExport::SigImageCallback(const sensor_msgs::ImageConstPtr &in_sig_image_msg)
{
  cv_bridge::CvImagePtr cv_sig_image = cv_bridge::toCvCopy(in_sig_image_msg, sensor_msgs::image_encodings::BGR8);
  sig_image_saver = cv_sig_image->image;
}

void ImageCloudDataExport::SaveImageFile(const cv::Mat &to_save_image, const std::string to_save_path){
  std::string string_counter = std::to_string(image_frame_counter_);
  std::string file_path = to_save_path + std::string(leading_zeros - string_counter.length(), '0') +
                          string_counter + ".jpg";

  cv::imwrite(file_path, to_save_image);
}


void ImageCloudDataExport::SyncedDataCallback(
  const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud,
  const sensor_msgs::CompressedImageConstPtr &in_image_msg)
  // const sensor_msgs::ImageConstPtr &in_image_msg)
{
  // 1. timesync data (2023.11.27 next time: use directly from in_image_msg's time, not from separately saved image's time)
  std::stringstream ss;
  std::string time_stamp_ss;
  ss << time_stamp_.sec << "." << std::setw(9) << std::setfill('0') << time_stamp_.nsec;
  time_stamp_ss = ss.str();
  ROS_INFO("[ImageCloudDataExport] Frame Synced: %s", time_stamp_ss.c_str());
  
  // 2. each data callback and save -> .jpg, .pcd
  LidarCloudCallback(in_sensor_cloud);
  ImageCallback(in_image_msg);
  SaveImageFile(ir_image_saver, path_ir_image_str_);
  SaveImageFile(refl_image_saver, path_refl_image_str_);
  SaveImageFile(range_image_saver, path_range_image_str_);
  SaveImageFile(sig_image_saver, path_sig_image_str_);

  // 3. timestamp saving
  std::string timestamp_path = path_timestamp_str_ + "timestamp.txt";
  std::ofstream outputFile(timestamp_path, std::ios::app);
  // outfile.open(timestamp_path);
  if (!outputFile.is_open()) {
    std::cerr << "Error: Unable to open the timestamp file." << std::endl;
  }
  outputFile << cloud_frame_counter_ << "," << time_stamp_ss << std::endl;
  outputFile.close();

  // increment counter
  cloud_frame_counter_++;
  image_frame_counter_++;
}

void ImageCloudDataExport::LidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*in_sensor_cloud, *cloud_ptr);

  std::string string_counter = std::to_string(cloud_frame_counter_);
  std::string file_path =
    path_pointcloud_str_ + std::string(leading_zeros - string_counter.length(), '0') + string_counter;

  /*std::ofstream pointcloud_file(file_path + ".txt");
  if (!pointcloud_file.fail())
  {
    pointcloud_file << cloud_ptr->points.size() << std::endl;
    for (size_t i = 0; i < cloud_ptr->points.size(); i++)
    {
      pointcloud_file << cloud_ptr->points[i].y << " " << cloud_ptr->points[i].z << " "
                      << cloud_ptr->points[i].x << " " << int(cloud_ptr->points[i].intensity)
                      << std::endl;
    }
    pointcloud_file.close();
  }*/
  //save PCD file as well
  // pcl::io::savePCDFileBinaryCompressed(file_path + ".pcd", *cloud_ptr);
  pcl::io::savePCDFileASCII(file_path + ".pcd", *cloud_ptr);
  if(!sync_topics_)
  {
    cloud_frame_counter_++;
  }
}

void ImageCloudDataExport::LidarCloudCallbackTimer(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud_timer)
{
  // resend pointcloud with saved image's time 

  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::fromROSMsg(*in_sensor_cloud_timer, *cloud_ptr);
  // TODO: time based on PC, not image

  sensor_msgs::PointCloud2 output;
  output = *in_sensor_cloud_timer;
  output.header.stamp = time_stamp_;
  cloud_sync_converter_pub_.publish(output);

}


void ImageCloudDataExport::Run()
{
  ros::NodeHandle private_node_handle("~");

  std::string image_src, points_src;
  std::string ir_image_src, refl_image_src, range_image_src, sig_image_src;
  private_node_handle.param<std::string>("image_src", image_src, "image_raw");
  ROS_INFO("[ImageCloudDataExport] image_src: %s", image_src.c_str());

  private_node_handle.param<std::string>("points_src", points_src, "points_raw");
  ROS_INFO("[ImageCloudDataExport] points_src: %s", points_src.c_str());

  private_node_handle.param<std::string>("ir_image_src", ir_image_src, "/ouster/nearir_image");
  ROS_INFO("[ImageCloudDataExport] ir_image_src: %s", ir_image_src.c_str());
  private_node_handle.param<std::string>("refl_image_src", refl_image_src, "/ouster/reflec_image");
  ROS_INFO("[ImageCloudDataExport] refl_image_src: %s", refl_image_src.c_str());
    private_node_handle.param<std::string>("range_image_src", range_image_src, "/ouster/range_image");
  ROS_INFO("[ImageCloudDataExport] range_image_src: %s", range_image_src.c_str());
    private_node_handle.param<std::string>("sig_image_src", sig_image_src, "/ouster/signal_image");
  ROS_INFO("[ImageCloudDataExport] sig_image_src: %s", sig_image_src.c_str());


  private_node_handle.param<bool>("sync_topics", sync_topics_, false);
  ROS_INFO("[ImageCloudDataExport] sync_topics: %d", sync_topics_);

  private_node_handle.param<bool>("ouster_use", ouster_use_, false);
  ROS_INFO("[ImageCloudDataExport] ouster_use: %d", ouster_use_);

  if(!sync_topics_)
  {
    cloud_sub_ = node_handle_.subscribe(points_src, 10, &ImageCloudDataExport::LidarCloudCallback, this);
    image_sub_ = node_handle_.subscribe(image_src, 10, &ImageCloudDataExport::ImageCallback, this);
  }
  else
  {
    if(ouster_use_)
    {
      std::string points_src_converted; 
      points_src_converted = points_src + "_converted";
      cloud_sync_sub_original_ = node_handle_.subscribe(points_src, 10, &ImageCloudDataExport::LidarCloudCallbackTimer, this);
      cloud_sync_converter_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2> (points_src_converted, 1);
      image_sub_ = node_handle_.subscribe(image_src, 10, &ImageCloudDataExport::ImageCallbackTimer, this);
      ir_image_sub_ = node_handle_.subscribe(ir_image_src, 10, &ImageCloudDataExport::IrImageCallback, this);
      refl_image_sub_ = node_handle_.subscribe(refl_image_src, 10, &ImageCloudDataExport::RefImageCallback, this);
      range_image_sub_ = node_handle_.subscribe(range_image_src, 10, &ImageCloudDataExport::RangeImageCallback, this);
      sig_image_sub_ = node_handle_.subscribe(sig_image_src, 10, &ImageCloudDataExport::SigImageCallback, this);

      // time sync subscriber
      cloud_sync_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                           points_src_converted,
                                                                           1);
    }
    else
    {
    // (2023.11.27) next time altogether with ir, ref, range, sig image fed to message_filters
    // time sync subscriber
      cloud_sync_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                           points_src,
                                                                           1);
    }

    // image_sync_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle_,
    //                                                                  image_src,
    //                                                                  1);
    image_sync_sub_ = new message_filters::Subscriber<sensor_msgs::CompressedImage>(node_handle_,
                                                                     image_src,
                                                                     1);
    
    // Main synchronizer definition using cloud_sync_sub and image_sync_sub //
    data_synchronizer_ =
        new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                       *cloud_sync_sub_,
                                                       *image_sync_sub_);
    data_synchronizer_->setMaxIntervalDuration(ros::Duration(0.1)); // https://answers.ros.org/question/284758/roscpp-message_filters-approximatetime-api-questions/
    data_synchronizer_->registerCallback(
        boost::bind(&ImageCloudDataExport::SyncedDataCallback, this, _1, _2));
  }

  time_t rawtime;
  struct tm *timeinfo;
  char buffer[80];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
  std::string datetime_str(buffer);

  const char *homedir;

  if ((homedir = getenv("HOME")) == NULL)
  {
    homedir = getpwuid(getuid())->pw_dir;
  }

  path_pointcloud_str_ = std::string(homedir) + "/output_" + datetime_str + "/point_clouds/";
  path_image_str_ = std::string(homedir) + "/output_" + datetime_str + "/images/CAM_FRONT/";
  path_timestamp_str_ = std::string(homedir) + "/output_" + datetime_str + "/timestamp/";

  path_ir_image_str_ = std::string(homedir) + "/output_" + datetime_str + "/ir_images/";
  path_refl_image_str_ = std::string(homedir) + "/output_" + datetime_str + "/refl_images/";
  path_range_image_str_ = std::string(homedir) + "/output_" + datetime_str + "/range_images/";
  path_sig_image_str_ = std::string(homedir) + "/output_" + datetime_str + "/signal_images/";

  leading_zeros = 6;

  boost::filesystem::path path_pointcloud(path_pointcloud_str_.c_str());
  boost::filesystem::path path_image(path_image_str_.c_str());
  boost::filesystem::path path_timestamp(path_timestamp_str_.c_str());
  boost::filesystem::path path_ir_image(path_ir_image_str_.c_str());
  boost::filesystem::path path_refl_image(path_refl_image_str_.c_str());
  boost::filesystem::path path_range_image(path_range_image_str_.c_str());
  boost::filesystem::path path_sig_image(path_sig_image_str_.c_str());


  boost::filesystem::create_directories(path_pointcloud);
  boost::filesystem::create_directories(path_image);
  boost::filesystem::create_directories(path_timestamp);
  boost::filesystem::create_directories(path_ir_image);
  boost::filesystem::create_directories(path_refl_image);
  boost::filesystem::create_directories(path_range_image);
  boost::filesystem::create_directories(path_sig_image);

  ROS_INFO("ImageCloudDataExport: PointCloud data stored in %s", path_pointcloud.c_str());
  ROS_INFO("ImageCloudDataExport: Image data stored in %s", path_image_str_.c_str()); 
  ROS_INFO("ImageCloudDataExport: IR Image data stored in %s", path_ir_image_str_.c_str()); 
  ROS_INFO("ImageCloudDataExport: Reflect Image data stored in %s", path_refl_image.c_str()); 
  ROS_INFO("ImageCloudDataExport: Range Image data stored in %s", path_range_image.c_str()); 
  ROS_INFO("ImageCloudDataExport: Signal Image data stored in %s", path_sig_image.c_str()); 

  ROS_INFO("ImageCloudDataExport: Timestamp data stored in %s", path_timestamp_str_.c_str());

  ROS_INFO("ImageCloudDataExport: Waiting for data...");
  ros::spin();

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_pointcloud_exporter");

  ImageCloudDataExport node;

  node.Run();

  return 0;
}
