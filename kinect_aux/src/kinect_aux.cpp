#include <libusb-1.0/libusb.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <tf/tf.h>

#include <map>
#include <vector>

// VID and PID for Kinect and motor/acc/leds
#define MS_MAGIC_VENDOR 0x45e
#define MS_MAGIC_MOTOR_PRODUCT 0x02b0
// Constants for accelerometers
#define GRAVITY 9.80665
#define FREENECT_COUNTS_PER_G 819.0
#define FREENECT_COUNTS_PER_G_X 850.0
#define FREENECT_COUNTS_PER_G_Y 850.0

#define FREENECT_COUNTS_PER_G_X_MAX 877.5
#define FREENECT_COUNTS_PER_G_X_ZERO 37.0
#define FREENECT_COUNTS_PER_G_X_MIN -795.0

#define FREENECT_COUNTS_PER_G_Y_MAX 861.0
#define FREENECT_COUNTS_PER_G_Y_ZERO 15.0
#define FREENECT_COUNTS_PER_G_Y_MIN -811.0

#define FREENECT_COUNTS_PER_G_Z_MAX 834.0
#define FREENECT_COUNTS_PER_G_Z_ZERO 3.0
#define FREENECT_COUNTS_PER_G_Z_MIN -823.5

#define ERROR_MARGIN	0.90

using namespace std;


struct Point2d {
	double x;
	double y;

	Point2d(){
		x = 0.0;
		y = 0.0;
	}

	Point2d(double x, double y){
		this->x = x;
		this->y = y;
	}
};

struct Line{
	Point2d start;
	Point2d end;
	double m;
	double b;

	Line(Point2d pt1, Point2d pt2){
		this->start = pt1;
		this->end = pt2;

		m = (start.y - end.y) / (start.x - end.x);
		b = end.y - (m * end.x);
	}

	void recompute(){
		m = (start.y - end.y) / (start.x - end.x);
		b = end.y - (m * end.x);
	}

	double getY(double x){
		return (m*x) + b;
	}
};

class Interpolator{
	public:
		vector<Line*> lines;

		Interpolator(){
			lines.clear();
		}

		Interpolator(Point2d start, Point2d end){
			lines.push_back(new Line(start, end) );
		}

		Interpolator(double x1, double y1, double x2, double y2){
			lines.push_back(new Line(Point2d(x1, y1), Point2d(x2, y2)) );
		}

		~Interpolator(){
			while(!lines.empty()){
				if(lines.back() != NULL){
					delete lines.back();
				}

				lines.pop_back();
			}
		}

		void addLine(Point2d pt1, Point2d pt2){
			lines.push_back(new Line(pt1, pt2) );
		}

		void recompute(){
			vector<Line*>::const_iterator lineIter = lines.begin();

			for(; lineIter != lines.end(); lineIter++){
				(*lineIter)->recompute();
			}
		}

		Line* getNearestLine(double x){
			vector<Line*>::const_iterator lineIter = lines.begin();
			Line* bestLine = NULL;
			double distance = -1;

			for(; lineIter != lines.end(); lineIter++){
				//Test start
				double startDelta = (*lineIter)->start.x - x;

				ROS_DEBUG("getNearestLine(%f): start delta = %f", x, startDelta);

				if(distance == -1 || abs(startDelta) < distance){
					distance = abs(startDelta);
					bestLine = *lineIter;
				}

				//Test end
				double endDelta = (*lineIter)->end.x - x;

				ROS_DEBUG("getNearestLine(%f): end delta = %f", x, endDelta);

				if(distance == -1 || abs(endDelta) < distance){
					distance = abs(endDelta);
					bestLine = *lineIter;
				}

				//Within bounds?
				if((startDelta < 0 && endDelta > 0) || (startDelta > 0 && endDelta < 0)){
					//Point falls directly within bounds, this is always the best line to select
					distance = abs(endDelta);
					bestLine = *lineIter;
				}
			}

			ROS_DEBUG("getNearestLine(%f): returned Line{ (%f, %f), (%f, %f) }", x, bestLine->start.x, bestLine->start.y, bestLine->end.x, bestLine->end.y);

			return bestLine;
		}

		double getValue(double x){
			Line* line = getNearestLine(x);
			double y = line->getY(x);

			//TODO: Want to filter here at all?

			return y;
		}
};

class Filter {
    public:
		unsigned int bufferLength;
		unsigned int mediansLength;

		vector<double> kernel;
		vector<double> data;
		vector<double> medians;
		multimap<double,double> medianMap;

		enum FilterType { Filter_Average, Filter_Median };

		uint8_t type;

		Filter(){
			bufferLength = 300;
			mediansLength = 50;
			data.clear();
			kernel.clear();
			medians.clear();

			type = Filter_Median;

		}

		Filter(int dataLength){
			bufferLength = dataLength;
			data.clear();
			kernel.clear();
			medians.clear();
		}

		double addKernel(double value){
			kernel.push_back(value);
		}

		double getValue(double value){


            if(type == Filter_Median){
                data.push_back(value);

				medianMap.clear();
				for(int i=0; i < data.size(); i++){
                    medianMap.insert(make_pair(data[i], data[i]));
                }

				double median = getValue();

				medians.push_back(median);

				if(medians.size() > mediansLength){
					medians.erase(medians.begin());
				}

				value = 0.0;
				for(unsigned int i=0; i<medians.size(); i++){
					value += medians[i];
				}

				value = value / (double) medians.size();
			}
			else if(type == Filter_Average){
				data.push_back(value);
				value = getValue();
			}


			if(data.size() > bufferLength){
				data.erase(data.begin());
			}


			return value;
		}

		double getValue(){
			double value = 0.0;

			if(type == Filter_Average){
				vector<double>::iterator kernelIter = kernel.begin();
				vector<double>::iterator dataIter = data.end();

				for(int i=0; i < data.size(); i++){
					double datam = data[i];

					if(kernel.size() != 0){
						value += datam * (*kernelIter);

						kernelIter++;

						if(kernelIter == kernel.end()){
							kernelIter = kernel.begin();
						}
					}
					else{
						value += datam;
					}
					dataIter--;
				}

				if(type==Filter_Average && kernel.size() == 0){
					value = value / (double) bufferLength;
				}
			}
			else if(type == Filter_Median){
				//select median value
				int middle = medianMap.size()/2;

				map<double,double>::iterator medianIter = medianMap.begin();
				for(int i=0; medianIter != medianMap.end(); medianIter++,i++){
					if(i==middle){
						value = medianIter->second;
						break;
					}
				}
			}

			return value;
		}
};

#define MIN_INDEX	0
#define ZERO_INDEX	1
#define MAX_INDEX	2

#define XAxisRanges { FREENECT_COUNTS_PER_G_X_MIN, FREENECT_COUNTS_PER_G_X_ZERO, FREENECT_COUNTS_PER_G_X_MAX }
#define YAxisRanges { FREENECT_COUNTS_PER_G_Y_MIN, FREENECT_COUNTS_PER_G_Y_ZERO, FREENECT_COUNTS_PER_G_Y_MAX }
#define ZAxisRanges { FREENECT_COUNTS_PER_G_Z_MIN, FREENECT_COUNTS_PER_G_Z_ZERO, FREENECT_COUNTS_PER_G_Z_MAX }

uint16_t selected_axis = 0;	//! Used by topic /imu/debug/axis_selector
double axes[3][3] = { XAxisRanges, YAxisRanges, ZAxisRanges };


Interpolator interpolators[3];
Filter filters[3];

// The kinect can tilt from +31 to -31 degrees in what looks like 1 degree increments
// The control input looks like 2*desired_degrees
#define MAX_TILT_ANGLE 31.0
#define MIN_TILT_ANGLE (-31.0)

ros::Publisher pub_imu;
ros::Publisher pub_tilt_angle;
ros::Publisher pub_tilt_status;

ros::Subscriber sub_tilt_angle;
ros::Subscriber sub_led_option;

ros::Subscriber sub_axis_selector;
ros::Subscriber sub_axis_max;
ros::Subscriber sub_axis_zero;
ros::Subscriber sub_axis_min;

libusb_device_handle *dev(0);

void openAuxDevice(int index = 0)
{
	libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	ssize_t cnt = libusb_get_device_list (0, &devs); //get the list of devices
	if (cnt < 0)
	{
		ROS_ERROR("No device on USB");
		return;
	}
	
	int nr_mot(0);
	for (int i = 0; i < cnt; ++i)
	{
		struct libusb_device_descriptor desc;
		const int r = libusb_get_device_descriptor (devs[i], &desc);
		if (r < 0)
			continue;

		// Search for the aux
		if (desc.idVendor == MS_MAGIC_VENDOR && desc.idProduct == MS_MAGIC_MOTOR_PRODUCT)
		{
			// If the index given by the user matches our camera index
			if (nr_mot == index)
			{
				if ((libusb_open (devs[i], &dev) != 0) || (dev == 0))
				{
					ROS_ERROR_STREAM("Cannot open aux " << index);
					return;
				}
				// Claim the aux
				libusb_claim_interface (dev, 0);
				break;
			}
			else
				nr_mot++;
		}
	}

	libusb_free_device_list (devs, 1);  // free the list, unref the devices in it
}

double lastRoll = 0.0;
double lastPitch = 0.0;
double lastForce = 0.0;
Filter deltaFilter;

void publishState(void)
{
	uint8_t buf[10];
	const int ret = libusb_control_transfer(dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 0);
	if (ret != 10)
	{
		ROS_ERROR_STREAM("Error in accelerometer reading, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
	
	const uint16_t ux = ((uint16_t)buf[2] << 8) | buf[3];
	const uint16_t uy = ((uint16_t)buf[4] << 8) | buf[5];
	const uint16_t uz = ((uint16_t)buf[6] << 8) | buf[7];
	
	const double accelerometer_x = (int16_t)ux;
	const double accelerometer_y = (int16_t)uy;
	const double accelerometer_z = ((int16_t)uz);
	const int8_t tilt_angle = (int8_t)buf[8];
	const uint8_t tilt_status = buf[9];
	
	// publish IMU
	sensor_msgs::Imu imu_msg;
	if (pub_imu.getNumSubscribers() > 0)
	{
		imu_msg.header.stamp = ros::Time::now();


		imu_msg.linear_acceleration.x = filters[0].getValue(interpolators[0].getValue(accelerometer_x) * GRAVITY);
		imu_msg.linear_acceleration.y = filters[1].getValue(interpolators[1].getValue(accelerometer_y) * GRAVITY);
		imu_msg.linear_acceleration.z = filters[2].getValue(interpolators[2].getValue(accelerometer_z) * GRAVITY);



		imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4]
			= imu_msg.linear_acceleration_covariance[8] = 0.01; // @todo - what should these be?
		imu_msg.angular_velocity_covariance[0] = -1; // indicates angular velocity not provided
		imu_msg.orientation_covariance[0] = -1; // indicates orientation not provided

		double magnitude = sqrt((imu_msg.linear_acceleration.x * imu_msg.linear_acceleration.x) + (imu_msg.linear_acceleration.y * imu_msg.linear_acceleration.y) + (imu_msg.linear_acceleration.z + imu_msg.linear_acceleration.z));

		//Only publish when the summed magnitued is within 15% of the expected force due to gravity
		double error = magnitude - GRAVITY;
		double delta = magnitude - lastForce;
        double deltaExpected = deltaFilter.getValue(delta);

		if(abs(delta - deltaExpected) < (0.1 * GRAVITY) && abs(error) < (GRAVITY * ERROR_MARGIN)){
			tf::Quaternion q;

			double pitch = -atan2( imu_msg.linear_acceleration.z, sqrt((imu_msg.linear_acceleration.x * imu_msg.linear_acceleration.x)  + (imu_msg.linear_acceleration.y * imu_msg.linear_acceleration.y)) );
			double roll = atan2( imu_msg.linear_acceleration.x, sqrt((imu_msg.linear_acceleration.z * imu_msg.linear_acceleration.z)  + (imu_msg.linear_acceleration.y * imu_msg.linear_acceleration.y)) );


            q.setRPY( roll, pitch, 0.0 );

			imu_msg.header.frame_id = "/camera_link";

			imu_msg.orientation.x = q.x();
			imu_msg.orientation.y = q.y();
			imu_msg.orientation.z = q.z();
			imu_msg.orientation.w = q.w();

			pub_imu.publish(imu_msg);
        }
	}
	
	// publish tilt angle and status
	if (pub_tilt_angle.getNumSubscribers() > 0)
	{
		std_msgs::Float64 tilt_angle_msg;
		tilt_angle_msg.data = double(tilt_angle) / 2.;
		pub_tilt_angle.publish(tilt_angle_msg);
	}
	if (pub_tilt_status.getNumSubscribers() > 0)
	{
		std_msgs::UInt8 tilt_status_msg;
		tilt_status_msg.data = tilt_status;
		pub_tilt_status.publish(tilt_status_msg);
	}
}


void setTiltAngle(const std_msgs::Float64 angleMsg)
{
	uint8_t empty[0x1];
	double angle(angleMsg.data);

	angle = (angle<MIN_TILT_ANGLE) ? MIN_TILT_ANGLE : ((angle>MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);
	angle = angle * 2;
	const int ret = libusb_control_transfer(dev, 0x40, 0x31, (uint16_t)angle, 0x0, empty, 0x0, 0);
	if (ret != 0)
	{
		ROS_ERROR_STREAM("Error in setting tilt angle, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
}

void setLedOption(const std_msgs::UInt16 optionMsg)
{
	uint8_t empty[0x1];
	const uint16_t option(optionMsg.data);
	
	const int ret = libusb_control_transfer(dev, 0x40, 0x06, (uint16_t)option, 0x0, empty, 0x0, 0);
	if (ret != 0)
	{
		ROS_ERROR_STREAM("Error in setting LED options, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
}

void setAxisSelectorCallback(const std_msgs::UInt16 axisId){
	selected_axis = axisId.data;
}

void setAxisMaxCallback(std_msgs::Float64 max){
	interpolators[selected_axis].lines[1]->end.x = max.data;

	interpolators[selected_axis].recompute();
}

void setAxisMinCallback(std_msgs::Float64 min){
	if(min.data > 0){
		interpolators[selected_axis].lines[0]->start.x = -min.data;
	}else{
		interpolators[selected_axis].lines[0]->start.x = min.data;
	}

	interpolators[selected_axis].recompute();
}

void setAxisZeroCallback(std_msgs::Float64 zero){

	if(selected_axis == 1){
		zero.data = -zero.data;
	}

	interpolators[selected_axis].lines[0]->end.x = zero.data;
	interpolators[selected_axis].lines[1]->start.x = zero.data;

	interpolators[selected_axis].recompute();
}

int main(int argc, char* argv[])
{
	int ret = libusb_init(0);
	if (ret)
	{
		ROS_ERROR_STREAM("Cannot initialize libusb, error: " << ret);
		return 1;
	}
	
	ros::init(argc, argv, "kinect_aux");
	ros::NodeHandle n;
	

	deltaFilter.type = Filter::Filter_Median;
	deltaFilter.bufferLength = 300;
	deltaFilter.mediansLength = 5;

    for(int i=0; i < 3; i++){
		Interpolator* interp = &interpolators[i];

		Point2d start(axes[i][0], -1.0);
		Point2d zero(axes[i][1], 0.0);
		Point2d end(axes[i][2], 1.0);


		interp->addLine( start, zero );
		interp->addLine( zero, end );
	}



	int deviceIndex;
	n.param<int>("device_index", deviceIndex, 0);
	openAuxDevice(deviceIndex);
	if (!dev)
	{
		ROS_ERROR_STREAM("No valid aux device found");
		libusb_exit(0);
		return 2;
	}
	
	pub_imu = n.advertise<sensor_msgs::Imu>("imu", 15);
	pub_tilt_angle = n.advertise<std_msgs::Float64>("cur_tilt_angle", 15);
	pub_tilt_status = n.advertise<std_msgs::UInt8>("cur_tilt_status", 15);
	
	sub_tilt_angle = n.subscribe("tilt_angle", 1, setTiltAngle);
	sub_led_option = n.subscribe("led_option", 1, setLedOption);

	sub_axis_selector = n.subscribe("/imu/debug/axis_selector", 1, setAxisSelectorCallback);
	sub_axis_min = n.subscribe("/imu/debug/axis_min", 1, setAxisMinCallback);
	sub_axis_zero = n.subscribe("/imu/debug/axis_zero", 1, setAxisZeroCallback);
	sub_axis_max = n.subscribe("/imu/debug/axis_max", 1, setAxisMaxCallback);

	while (ros::ok())
	{
		ros::spinOnce();
		publishState();
	}
	
	libusb_exit(0);
	return 0;
}
