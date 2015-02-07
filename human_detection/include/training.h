// training montage

class training_collect
{
public:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::SubscriberFilter sub_img_depth;
	image_transport::SubscriberFilter sub_img_colour;
	int frame_count;

	ros::Subscriber sub_key;
	bool key_pressed;

	image_transport::Publisher pub_out;

	training_collect();

	void keypressCallback(const std_msgs::String::ConstPtr& msg);

	void imageCallback(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::ImageConstPtr& colour_image);
	
	typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image, sensor_msgs::Image > DepthSyncPolicy; 

	message_filters::Synchronizer< DepthSyncPolicy > sync; 

};
