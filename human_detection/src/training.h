// training montage

class training_collect
{
public:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_in;
	image_transport::Subscriber sub_img;
	int frame_count;

	ros::Subscriber sub_key;
	bool key_pressed;

	image_transport::ImageTransport it_out;
	image_transport::Publisher pub_out;

	training_collect();

	void keypressCallback(const std_msgs::String::ConstPtr& msg);

	void imageCallback(const sensor_msgs::ImageConstPtr& original_image);
};
