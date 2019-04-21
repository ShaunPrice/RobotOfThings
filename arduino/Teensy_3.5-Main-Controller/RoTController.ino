/*
    Name:       RoTController.ino
    Created:	17/12/2018 7:49:41 PM
    Author:     SHAUNSPC\Shaun Price
*/

/*
################################################################
  TO WORK ON Teensy3.5/3.6
################################################################
Note: rosserial updates may overwrite these changes
Add this to the top of ArduinoHardware.h
https://github.com/nr-parikh/rosserial_tutorials/tree/master/misc_files

#if defined(__arm__) && defined(CORE_TEENSY)
	#include <usb_serial.h>
	#define SERIAL_CLASS usb_serial_class
#endif

and comment out from ArduinoIncludes.h:
  //#define SERIAL_CLASS HardwareSerial

also, in ros.h changed the following to increase the buffer from 512 to 1024:
  typedef NodeHandle_<ArduinoHardware, 25, 25, 1024, 1024> NodeHandle;

################################################################
################################################################
########### Disable the TIC's Pullup resistors in the ##########
########### advanced settings for 3.3V devices.       ##########
################################################################*/

#include <Audio.h>
#include <SerialFlash.h>
#include <SD.h>
#include <Wire.h>

// ROS Includes
#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h> 
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs\JointState.h>
#include <sensor_msgs\LaserScan.h>

////////////////////////////////////////////////////////////////
// rosserial_arduino generated headers
// Use to generate and add to rosserial_arduino headers
// rosrun rosserial_arduino make_libraries.py <sketckbook_output_folder>
////////////////////////////////////////////////////////////////
#include <ackermann_msgs/AckermannDriveStamped.h>
////////////////////////////////////////////////////////////////

#include <FastLED.h>
#include <Tic.h>
#include <Adafruit_PWMServoDriver.h>
#include <TFmini.h>
#include <MPU9250.h>
#include "quaternionFilters_modified.h"
#include <ArduinoJson.h>
#include "imumaths/imumaths.h"

#define BRAKE_DISTANCE 0.15 // Distance in meters from the front or rear ultrasonic sensors that be need to stop 
#define MAIN_LOOP_HZ 20
#define IMU_LOOP_HZ  100 // Frequency the IMU Loop is read

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define SERVO_COUNT 3
#define STEERING	0
#define HEAD_PAN	1
#define HEAD_TILT	2

#define MOTOR_REVERSE -100000000 // Motor is in steps per 10,000 seconds
#define MOTOR_STOP 0
#define MOTOR_FORWARD +100000000 // Motor is in steps per 10,000 seconds

#define STEERING_RANGE	 20
#define STEERING_RIGHT	100
#define STEERING_FORWARD 80
#define STEERING_LEFT	 60

#define HEAD_PAN_RANGE	 40
#define HEAD_PAN_RIGHT	110
#define HEAD_PAN_FORWARD 70
#define HEAD_PAN_LEFT	 30

#define HEAD_TILT_RANGE  40
#define HEAD_TILT_UP	125
#define HEAD_TILT_LEVEL  85
#define HEAD_TILT_DOWN	 45

#define SERIAL1_RX_PIN 0
#define SERIAL1_TX_PIN 1

#define SERIAL5_RX_PIN 34
#define SERIAL5_TX_PIN 33
#define SERIAL5_BAUD_RATE 115200

#define I2C0_SDA_PIN 18
#define I2C0_SCL_PIN 19

#define I2C1_SDA_PIN 38
#define I2C1_SCL_PIN 37

#define I2C2_SDA_PIN 4
#define I2C2_SCL_PIN 3

#define FRONT_ULTRASONIC_TRIG_PIN 26
#define FRONT_ULTRASONIC_ECHO_PIN 27
#define REAR_ULTRASONIC_TRIG_PIN 28
#define REAR_ULTRASONIC_ECHO_PIN 29

#define PI_NUM_LEDS 6
#define PI_LED_DATA_PIN 2

#define ODROID_NUM_LEDS 6
#define ODROID_LED_DATA_PIN 25

#define MOUTH_NUM_LEDS 3
#define MOUTH_LED_DATA_PIN 6

#define LIDAR_HEAD_PAN_STEPS 20 // Steps per scan
#define LIDAR_HEAD_PAN_STEP 200 / LIDAR_HEAD_PAN_STEPS // 200 (-100% to 100%) divided by the # steps
#define LIDAR_MIN_RANGE 0.3
#define LIDAR_MAX_RANGE 12.0
#define LIDAR_FIELD_OF_VIEW 0.04
#define LIDAR_MIN_INTENSITY 20

// AudioMemory in Teensy Audio ligrary has issues.
// Define the memory bock manually
#define MEMORY_BLOCK_SIZE 12
static DMAMEM audio_block_t audioMemoryBlock[MEMORY_BLOCK_SIZE];

// GUItool: begin automatically generated code
AudioInputUSB           usb1;           //xy=131,330wwaal
AudioAnalyzePeak        peak1;          //xy=471,368
AudioAnalyzeFFT256		fft1;
AudioMixer4				mixer1;
AudioOutputAnalog		dac0;
AudioConnection         patchCord1(usb1, 0, mixer1, 0);
AudioConnection         patchCord2(usb1, 1, mixer1, 1);
AudioConnection         patchCord3(mixer1, 0, dac0, 0);
AudioConnection         patchCord4(mixer1, 0, peak1, 0);
AudioConnection         patchCord5(mixer1, 0, fft1, 0);
// GUItool: end automatically generated code

bool musicLightsMode = false;

volatile bool lidarScanMode = false;
enum LidarScanState {Start = 0, Initialising = 1, BeginScanning, Scanning, EndScanning};
volatile int lidarScanState = LidarScanState::Start;
volatile uint32_t lidarInitialiseWait = 0;
volatile uint16_t lidar_counter = 0;
volatile int16_t scan_angle = 0;
sensor_msgs::LaserScan lidar_scan_msg;
sensor_msgs::LaserScan temp_lidar_scan_msg;
volatile bool sendLidarMsg = false;

CRGB piLeds[PI_NUM_LEDS];
CRGB odroidLeds[ODROID_NUM_LEDS];
CRGB mouthLeds[MOUTH_NUM_LEDS];
uint8_t gHue = 0; // rotating "base color"

IntervalTimer imuTimer;
IntervalTimer ledUpdateTimer;
IntervalTimer rangeTimer;
IntervalTimer lidarTimer;

MPU9250 bodyImu(MPU9250_ADDRESS_AD0, Wire2);

QuaternionFilters bodyImuFilter;

struct ServoLimits
{
	uint16_t min;
	uint16_t middle;
	uint16_t max;
};

bool logSDEnable = true;
bool lidarActive = false;

bool ledState = false; // Indicator to let us know the updates are running

ServoLimits rotServoLimits[SERVO_COUNT] = { { STEERING_LEFT, STEERING_FORWARD , STEERING_RIGHT },
											{ HEAD_PAN_RIGHT, HEAD_PAN_FORWARD, HEAD_PAN_LEFT }, 
											{ HEAD_TILT_DOWN, HEAD_TILT_LEVEL, HEAD_TILT_UP } };

volatile float32_t lidarRange = 0;
volatile float32_t lidarIntensity = 0;
volatile uint32_t lidarUpdateTime = 0;
volatile float32_t motorCurrentSpeedVal = 0;
volatile float32_t motorCurrentPositionVal = 0; // Value for the current motor position reading
volatile uint32_t motorCurrentPositionValTime = 0; // Timestamp for the current motor position reading
volatile double_t motorPreviousPositionVal = 0; // Value for the previous motor position reading
volatile uint32_t motorPreviousPositionValTime = 0; // Timestamp for the previous motor position reading
volatile bool motorEmergencyStopVal = false;

// ROS Ackermann_msg data 
volatile float32_t speed = 0; // desired forward speed (m/s)
volatile float32_t acceleration = 0; // # NOT USED # desired acceleration(m / s ^ 2)
volatile float32_t jerk = 0; // # NOT USED # desired jerk (m/s^3)
volatile float32_t steering_angle = 0; // desired virtual angle (radians)
volatile float32_t steering_angle_velocity = 0; // # NOT USED # desired rate of change (radians/s)

// Braking if too close
volatile bool tooCloseSet = false; // Used to indicate the ultrasonics have already stopped the motors
volatile float32_t previousSpeed = 0.0; // Used to store the speed prior to the ultrasonic sensors tell the motors to stop


float32_t odom_x = 0;
float32_t odom_y = 0;

volatile float32_t headPanRadians = 0;
volatile float32_t headTiltRadians = 0;

volatile uint32_t frontUltrasonicTimestamp = 0; // milliseconds range recorded
volatile float32_t frontUltrasonicRange = 0;

volatile uint32_t rearUltrasonicTimestamp = 0; // milliseconds range recorded
volatile float32_t rearUltrasonicRange = 0;

uint32_t odom_current_millis, odom_last_millis;

char* jointStateNames[] = { "head_pan_link", "head_tilt_link", "steeringLeft", "steeringRight" };
const char* odometry_id = "odom";
const char* base_link_id = "base_link";
const char* imu_body_link_id = "imu_body_link";
const char* head_pan_link_id = "head_pan_link";
const char* head_tilt_link_id = "head_tilt_link";
const char* joint_states_id = "joint_states";

const char* imu_body_id = "imu_body";
const char* lidar_id = "lidar";
const char* ultrasonic_front_id = "ultrasonic_front";
const char* ultrasonic_rear_id = "ultrasonic_rear";

const char* motorCurrentSpeed_id = "motorCurrentSpeed";
const char* motorCurrentPosition_id = "motorCurrentPosition";
const char* motorEmergencyStop_id = "motorEmergencyStop";
const char* motorCurrentOperationState_id = "motorCurrentOperationState";

const char* headTilt_id = "headTilt";
const char* headPan_id = "headPan";
const char* motorSpeed_id = "speed";
const char* motorPosition_id = "motorPosition";
const char* motorEmergencyStopSetting_id = "motorEmergencyStopSetting";
const char* motorEnergise_id = "motorEnergise";
const char* steering_id = "steering_angle";
const char* motorOperationState_id = "motorOperationState";
const char* musicLights_id = "musicLights";
const char* lidar_scan_enable_id = "lidarScan";
const char* lidar_scan_id = "laser_scan";
const char* ackermann_id = "ackermann_cmd";

ros::NodeHandle rosNodeHandler;
tf::TransformBroadcaster transformBroadcaster;

geometry_msgs::TransformStamped transformOdometry;
geometry_msgs::TransformStamped transformBase;
geometry_msgs::TransformStamped transformUltrasonicFront;
geometry_msgs::TransformStamped transformUltrasonicRear;

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver();

TicI2C tic;

TFmini  frontLidar;

String filename = "";

unsigned long microsPerReading, microsPrevious;

void logSD(String log)
{
	if (logSDEnable)
	{
		// Log string to SD
		File logFile = SD.open(filename.c_str(), FILE_WRITE);

		// if the file is available, write to it:
		if (logFile)
		{
			logFile.println(log);
			logFile.close();
		}
	}
}

void processLEDLoop()
{
	CRGB mouthColour = CRGB::White;

	CRGB odroidColour = CRGB::Blue;
	CRGB piColour = CRGB::Blue;

	uint8_t level = 0x00;
	uint8_t red = 0x00;
	uint8_t green = 0x00;
	uint8_t blue = 0x00;
	uint8_t voice = 0x00;

	// Check if ROS is connected. Turn Odroid yellow if not
	if (rosNodeHandler.connected())
	{
		// Only run if the USB Audio is active
		if (usb1.isActive())
		{
			if (peak1.available())
			{
				float peak = peak1.read();
				level = (uint8_t)((peak < 0.25) ? 0x00 : peak * 0xFF);
			}

			if (musicLightsMode)
			{
				mouthColour = CRGB::Black;
				odroidColour = CRGB::Black;
				piColour = CRGB::Black;

				if (fft1.available())
				{
					float f_voice = fft1.read(3, 10);
					float f_low = fft1.read(1, 2);
					float f_mid = fft1.read(3, 50);
					float f_high = fft1.read(51, 127);

					// Make the output logarithmic
					red = (uint8_t)(pow(f_low, 1.5) / 16 * level * 0xFF);
					green = (uint8_t)(pow(f_mid, 1.5) / 16 * level * 0xFF);
					blue = (uint8_t)(pow(f_high, 1.5) / 16 * level * 0xFF);

					voice = (uint8_t)(f_voice * f_voice / 256 * level * 0xFF);
				}
				else
				{
					voice = 0x00;
					red = 0x00;
					green = 0x00;
					blue = 0x00;
				}

				// Odroid is the beat/bass
				odroidColour.red = red;

				// Pi mid frequency colour
				piColour.green = green;

				// Pi High frequency colour
				piColour.blue = blue;

				// Voice tones
				mouthColour.red = voice;
			}
			else
			{
				if (level != 0)
				{
					mouthColour.red = level;
					mouthColour.green = 0;
					mouthColour.blue = 0;
				}
				else
				{
					mouthColour = CRGB::White;
				}
			}
		}
		else
		{
			mouthColour = CRGB::White;
		}
	}
	else
	{
		// Odroid indicating ROS not connected
		odroidColour = CRGB::Yellow;
	}
	

	fill_solid(piLeds, PI_NUM_LEDS, piColour);
	fill_solid(odroidLeds, ODROID_NUM_LEDS, odroidColour);
	fill_solid(mouthLeds, MOUTH_NUM_LEDS, mouthColour);

	// Update the lights
	FastLED.show();
}

// String to char* utility
char* string2char(String command)
{
	if (command.length() != 0) {
		char *p = const_cast<char*>(command.c_str());
		return p;
	}
	else
		return nullptr;
}

// Resets the TIC motor controller command timeout
// and runs the spinOnce function of RoS
void spinResetCommandTimeout()
{
	rosNodeHandler.spinOnce();
	tic.resetCommandTimeout();
	delay(1);
}

//#################################################################################################
// RoS publisher routines
//#################################################################################################

// RoS publisher data
nav_msgs::Odometry odometry_msg;
sensor_msgs::Range lidar_msg;
sensor_msgs::Range ultrasonic_front_msg;
sensor_msgs::Range ultrasonic_rear_msg;
std_msgs::Float32 motorCurrentSpeed_msg;
std_msgs::Float32 motorCurrentPosition_msg;
std_msgs::String motorEmergencyStop_msg;
std_msgs::String motorCurrentOperationState_msg;
sensor_msgs::Imu bodyImu_msg;
sensor_msgs::JointState joint_state_msg;
sensor_msgs::LaserScan laser_scan_msg;

// RoS Publishers
ros::Publisher pubOdometry(odometry_id, &odometry_msg);
ros::Publisher pubLidar(lidar_id, &lidar_msg);
ros::Publisher pubLidarScan(lidar_scan_id, &lidar_scan_msg);
ros::Publisher pubUltrasonicFront(ultrasonic_front_id, &ultrasonic_front_msg);
ros::Publisher pubUltrasonicRear(ultrasonic_rear_id, &ultrasonic_rear_msg);
ros::Publisher pubMotorCurrentSpeed(motorCurrentSpeed_id, &motorCurrentSpeed_msg);
ros::Publisher pubMotorCurrentPosition(motorCurrentPosition_id, &motorCurrentPosition_msg);
ros::Publisher pubMotorEmergencyStop(motorEmergencyStop_id, &motorEmergencyStop_msg);
ros::Publisher pubMotorCurrentOperationState(motorCurrentOperationState_id, &motorCurrentOperationState_msg);
ros::Publisher pubBodyImu(imu_body_id, &bodyImu_msg);
ros::Publisher pubJointState(joint_states_id, &joint_state_msg);

void publishLidar()
{
	if (micros() - lidarUpdateTime <= 100000)
	{
		lidar_msg.header.frame_id = lidar_id;
		lidar_msg.header.stamp = rosNodeHandler.now();

		// Check if its had an update since last time.
		// If it hasn't set the lidar to 0.
		lidar_msg.range = lidarRange / 100;
		lidar_msg.field_of_view = LIDAR_FIELD_OF_VIEW;
		lidar_msg.max_range = LIDAR_MAX_RANGE;
		lidar_msg.min_range = LIDAR_MIN_RANGE;
		lidar_msg.radiation_type = lidar_msg.INFRARED;

		pubLidar.publish(&lidar_msg);
	}
}

void publishLidarScan()
{
	if (sendLidarMsg)
	{
		sendLidarMsg = false;

		pubLidarScan.publish(&lidar_scan_msg);
	}
}

void publishJointState()
{
	joint_state_msg.header.frame_id = joint_states_id;
	joint_state_msg.header.stamp = rosNodeHandler.now();

	joint_state_msg.name_length = 4;
	joint_state_msg.position_length = 4;
	float32_t positions[4];
	positions[0] = headPanRadians;
	positions[1] = headTiltRadians;
	positions[2] = steering_angle;
	positions[3] = steering_angle;
	joint_state_msg.name = jointStateNames;
	joint_state_msg.position = positions;

	pubJointState.publish(&joint_state_msg);
}

void publishFrontUltrasonicRange()
{
	// Ultrasonic is within range so publish
	ultrasonic_front_msg.header.stamp = rosNodeHandler.now();
	ultrasonic_front_msg.header.frame_id = ultrasonic_front_id;

	if ((millis() - frontUltrasonicTimestamp) <= 100)
		ultrasonic_front_msg.range = frontUltrasonicRange;
	else
		ultrasonic_front_msg.range = 0;

	ultrasonic_front_msg.field_of_view = 0.26;
	ultrasonic_front_msg.max_range = 4.5;
	ultrasonic_front_msg.min_range = 0.02;
	ultrasonic_front_msg.radiation_type =  ultrasonic_front_msg.ULTRASOUND;

	pubUltrasonicFront.publish(&ultrasonic_front_msg);
}

void publishRearUltrasonicRange()
{
	// Ultrasonic is within range so publish
	ultrasonic_rear_msg.header.stamp = rosNodeHandler.now();
	ultrasonic_rear_msg.header.frame_id = ultrasonic_rear_id;

	if ((millis() - rearUltrasonicTimestamp) <= 100)
		ultrasonic_rear_msg.range = rearUltrasonicRange;
	else
		ultrasonic_rear_msg.range = 0;

	ultrasonic_rear_msg.field_of_view = 0.26;
	ultrasonic_rear_msg.max_range = 4.5;
	ultrasonic_rear_msg.min_range = 0.02;
	ultrasonic_rear_msg.radiation_type = ultrasonic_rear_msg.ULTRASOUND; 

	pubUltrasonicRear.publish(&ultrasonic_rear_msg);
}

// Motor odometry publisher (meters)
void publishMotorCurrentPosition()
{
	motorPreviousPositionVal = motorCurrentPositionVal;
	motorPreviousPositionValTime = motorCurrentPositionValTime;
	
	motorCurrentPositionVal = (float)tic.getCurrentPosition() / 9000.00 * 0.22305;
	motorCurrentPositionValTime = millis();

	motorCurrentPosition_msg.data = motorCurrentPositionVal;
	pubMotorCurrentPosition.publish(&motorCurrentPosition_msg);
}

void publishMotorCurrentSpeed()
{
	// convert from microsteps per 10,000 seconds meters/sec 
	motorCurrentSpeedVal = (float)tic.getCurrentVelocity() / 407219470.60;
	motorCurrentSpeed_msg.data = motorCurrentSpeedVal;
	pubMotorCurrentSpeed.publish(&motorCurrentSpeed_msg);
}

void publishOdometry()
{
	ros::Time timestamp = rosNodeHandler.now();

	odom_current_millis = millis();
	
	// Calculate the distance moved
	float32_t odom_diff = (motorCurrentPositionVal - motorPreviousPositionVal); // Convert to meters

	// Convert from odometry to X, Y
	float32_t w = transformBase.transform.rotation.w;
	float32_t x = transformBase.transform.rotation.x;
	float32_t y = transformBase.transform.rotation.y;
	float32_t z = transformBase.transform.rotation.z;
	
	// Calculate components of yaw, pitch, roll
	
	float32_t yaw = -atan(2.0 * (x*y + w*z));
	//float32_t pitch = asin(-2.0 * (w*y - x * z));
	//float32_t roll = atan((2.0 * (w*x + y * z)) / (z*z - y * y - x * x + w * w));

	float32_t odom_dx = -acos(yaw) * odom_diff;
	float32_t odom_dy = asin(yaw) * odom_diff;

	transformOdometry.header.frame_id = odometry_id;
	transformOdometry.header.stamp = timestamp;
	transformOdometry.child_frame_id = base_link_id;

	odometry_msg.header.frame_id = odometry_id;
	odometry_msg.header.stamp = timestamp;
	odometry_msg.child_frame_id = base_link_id;


	// Make sure we're not publishing bad data
	if (!isnan(odom_dx) && !isnan(odom_dy))
	{
		odom_x += odom_dx;
		odom_y += odom_dy;

		transformOdometry.transform.translation.x = odom_x;
		transformOdometry.transform.translation.y = odom_y;
		transformOdometry.transform.translation.z = 0.04;

		transformOdometry.transform.rotation.w = sin(yaw/2);
		transformOdometry.transform.rotation.x = 0.0;
		transformOdometry.transform.rotation.y = 0.0;
		transformOdometry.transform.rotation.z = cos(yaw / 2);

		odometry_msg.pose.pose.position.x = odom_x;
		odometry_msg.pose.pose.position.y = odom_y;
		odometry_msg.pose.pose.position.z = 0.0;
		odometry_msg.pose.pose.orientation.w = sin(yaw/2);
		odometry_msg.pose.pose.orientation.x = 0.0;
		odometry_msg.pose.pose.orientation.y = 0.0;
		odometry_msg.pose.pose.orientation.z = cos(yaw / 2);

		odometry_msg.twist.twist.linear.x = odom_dx;
		odometry_msg.twist.twist.linear.y = odom_dy;
		odometry_msg.twist.twist.linear.z = 0;

		odometry_msg.twist.twist.angular.x = 0;
		odometry_msg.twist.twist.angular.y = -yaw;
		odometry_msg.twist.twist.angular.z = 0;
	}
		
	transformBroadcaster.sendTransform(transformOdometry);
	pubOdometry.publish(&odometry_msg);

	odom_last_millis = odom_current_millis;
}

void publishMotorEmergencyStop(bool state, const char* message)
{
	if (state)
		tic.setTargetVelocity(0);

	motorEmergencyStopVal = state;
	motorEmergencyStop_msg.data = message;
	pubMotorEmergencyStop.publish(&motorEmergencyStop_msg);
}

void publishMotorCurrentOperationState()
{
	// Check if we're in a safestart violation
	TicOperationState opState = tic.getOperationState();

	switch (opState)
	{
	case TicOperationState::Deenergized:
		motorCurrentOperationState_msg.data = "Motor Operation State: Deenergized";
		logSD("Motor Operation State : Deenergized");
		break;
	case TicOperationState::Normal:
		motorCurrentOperationState_msg.data = "Motor Operation State: Normal";
		logSD("Motor Operation State : Normal");
		break;
	case TicOperationState::Reset:
		motorCurrentOperationState_msg.data = "Motor Operation State: Reset";
		logSD("Motor Operation State : Reset");
		break;
	case TicOperationState::SoftError:
		motorCurrentOperationState_msg.data = "Motor Operation State: Soft Error";
		logSD("Motor Operation State : Soft Error");
		break;
	case TicOperationState::StartingUp:
		motorCurrentOperationState_msg.data = "Motor Operation State: Starting Up";
		logSD("Motor Operation State : Starting Up");
		break;
	case TicOperationState::WaitingForErrLine:
		motorCurrentOperationState_msg.data = "Motor Operation State: Waiting for Err Line";
		logSD("Motor Operation State : Waiting for Err Line");
		break;
	default:
		motorCurrentOperationState_msg.data = "Motor Operation State: Unknown";
		logSD("Motor Operation State : Unknown");
	}
	
	pubMotorCurrentOperationState.publish(&motorCurrentOperationState_msg);
}

void publishBodyImu()
{
	bodyImu_msg.header.stamp = rosNodeHandler.now();
	bodyImu_msg.header.frame_id = imu_body_id;

	bodyImu_msg.orientation.w = bodyImuFilter.getQ()[0];
	bodyImu_msg.orientation.x = bodyImuFilter.getQ()[1];
	bodyImu_msg.orientation.y = bodyImuFilter.getQ()[2];
	bodyImu_msg.orientation.z = bodyImuFilter.getQ()[3];

	bodyImu_msg.orientation_covariance[0] = 0.0025;
	bodyImu_msg.orientation_covariance[4] = 0.0025;
	bodyImu_msg.orientation_covariance[8] = 0.0025;

	bodyImu_msg.angular_velocity.x = bodyImu.gx;
	bodyImu_msg.angular_velocity.y = bodyImu.gy;
	bodyImu_msg.angular_velocity.z = bodyImu.gz;
	
	bodyImu_msg.angular_velocity_covariance[0] = 0.0025;
	bodyImu_msg.angular_velocity_covariance[4] = 0.0025;
	bodyImu_msg.angular_velocity_covariance[8] = 0.0025;

	bodyImu_msg.linear_acceleration.x = bodyImu.ax;
	bodyImu_msg.linear_acceleration.y = bodyImu.ay;
	bodyImu_msg.linear_acceleration.z = bodyImu.az;

	bodyImu_msg.linear_acceleration_covariance[0] = 0.0025;
	bodyImu_msg.linear_acceleration_covariance[4] = 0.0025;
	bodyImu_msg.linear_acceleration_covariance[8] = 0.0025;
	
	transformBase.header.frame_id = base_link_id;
	transformBase.header.stamp = rosNodeHandler.now();
	transformBase.child_frame_id = imu_body_id;
	transformBase.transform.translation.x = 0.0;
	transformBase.transform.translation.y = 0.0;
	transformBase.transform.translation.z = 0.14;
	transformBase.transform.rotation.w = bodyImu_msg.orientation.w;
	transformBase.transform.rotation.x = bodyImu_msg.orientation.x;
	transformBase.transform.rotation.y = bodyImu_msg.orientation.y;
	transformBase.transform.rotation.z = bodyImu_msg.orientation.z;

	pubBodyImu.publish(&bodyImu_msg);
	transformBroadcaster.sendTransform(transformBase);
}

unsigned int scaleData(int data, int min, int centre, int max)
{
	if (data >= centre)
		return map(data, 0, 100, centre, max);
	else
		return map(data, -100, 0, min, centre);
}


void servoMove(uint8_t servo, int16_t percent)
{
	// Check the range
	if (percent > 100 || percent < -100)
	{
		logSD("Servo setting of " + String(percent) + "out of range (-100 to 100). Contrained to limits.");
		percent = constrain(percent, -100, 100);
	}

	uint16_t scaledDegrees = scaleData(percent, rotServoLimits[servo].min, rotServoLimits[servo].middle, rotServoLimits[servo].max);
	servos.setPWM(servo, 0, map(scaledDegrees, 0, 180, SERVOMIN, SERVOMAX));

	logSD("servoMove Number: " + String(servo));
	logSD("          Percent:" + String(percent));
}

// Steering ROS Handler
void steering_cb(const std_msgs::Float32& cmd_msg)
{
	// Convert radians to -100% to 100%
	int16_t percent = static_cast<int16_t>(-cmd_msg.data / (STEERING_RANGE / (180.0 / PI))*100);

	servoMove(STEERING, percent);
	rosNodeHandler.loginfo("steering set");
	logSD("steering: " + String((float32_t)cmd_msg.data));
}

// Head Tilt RoS Handler
void headTilt_cb(const std_msgs::Int16& cmd_msg)
{
	if (!lidarScanMode)
	{
		servoMove(HEAD_TILT, cmd_msg.data);
		rosNodeHandler.loginfo("headTilt set");
		headTiltRadians = cmd_msg.data * -0.006981; // convert % to rad
		logSD("headTilt: " + String((int16_t)cmd_msg.data));
	}
	else
	{
		logSD("headTilt command inactive. In LIDAR scan mode");
	}
}

// Head Pan RoS Handler
void headPan_cb(const std_msgs::Int16& cmd_msg)
{
	if (!lidarScanMode)
	{
		servoMove(HEAD_PAN, cmd_msg.data);
		rosNodeHandler.loginfo("headPan set");
	
		headPanRadians = cmd_msg.data * -0.006981; // convert % to rad
		logSD("headPan: " + String((int16_t)cmd_msg.data));
	}
	else
	{
		logSD("headPan command inactive. In LIDAR scan mode");
	}
}

// Command handler for setting and resetting the motor emergency stop
void motorEmergencyStop_cb(const std_msgs::Bool& cmd_msg)
{
	if (cmd_msg.data == true)
	{
		publishMotorEmergencyStop(true, "Motor Emergency Stop - ROS Command");
		rosNodeHandler.loginfo("Motor Emergency Stop - ROS Command");
		logSD("Motor Emergency Stop - ROS Command");
	}
	else
	{
		publishMotorEmergencyStop(false, "Motor Emergency Stop Reset - ROS Command");
		rosNodeHandler.loginfo("Motor Emergency Stop Reset - ROS Command");
		logSD("Motor Emergency Stop Reset - ROS Command");
	}
}

// Command handler for energising and de-energising the motor
void motorEnergise_cb(const std_msgs::Bool& cmd_msg)
{
	logSD("motorEnergise: " + String((bool)cmd_msg.data));

	if (cmd_msg.data == true)
	{
		tic.energize();
		tic.clearDriverError();

		if (tic.getEnergized())
			rosNodeHandler.loginfo("Motor Energised");
		else
		{
			rosNodeHandler.loginfo("Error energising motor - Not Energised");
			logSD("Error energising motor - Not Energised");
		}
	}
	else
	{
		tic.deenergize();

		if (!tic.getEnergized())
			rosNodeHandler.loginfo("Motor Deenergised");
		else
			rosNodeHandler.loginfo("Error deenergising motor");
	}
}

// Robot speed (speed) in m/s
void motorSpeed_cb(const std_msgs::Float32& cmd_msg)
{
	logSD("motorSpeed: " + String((int32_t)cmd_msg.data));

	// Convert m/s to -100% to 100%
	int32_t speed = static_cast<int32_t>(cmd_msg.data*407.332);

	// Check the range
	if (speed > 100 || speed < -100)
	{
		logSD("Motor Speed setting of " + String(speed) + "out of range (-100 to 100). Contrained to limits.");
		speed = constrain(speed, -100, 100);
	}

	// Check if we're in a safestart violation
	if (tic.getErrorStatus() & (uint16_t)TicError::SafeStartViolation)
	{
		// We are so clear it
		tic.exitSafeStart();
		rosNodeHandler.loginfo("motorSpeed Safe Start reset");
	}

	if (!motorEmergencyStopVal)
	{
		int32_t motorSpeed = (int32_t)speed * 1000000; //Convert to steps per 10000 seconds by multiplying by 1,000,000
		tic.setTargetVelocity(motorSpeed);
		String message = "motorSpeed set to " + String(cmd_msg.data);
		rosNodeHandler.loginfo(string2char(message));
	}
	else
	{
		rosNodeHandler.loginfo("motorSpeed not set. Emergency Stop state.");
		logSD("ERROR: motorSpeed not set. Emergency Stop state.");
	}
}

void motorPosition_cb(const std_msgs::Float32& cmd_msg)
{
	logSD("motorPosition: " + String((float32_t)cmd_msg.data,3));
	
	// Check if we're in a safestart violation
	if (tic.getErrorStatus() & (uint16_t)TicError::SafeStartViolation)
	{
		// We are so clear it
		tic.exitSafeStart();
		rosNodeHandler.loginfo("motorPosition Safe Start reset");
	}

	if (!motorEmergencyStopVal)
	{
		float32_t motorPosition = (float32_t)cmd_msg.data * 9000.00 / 0.22305;;
		tic.setTargetPosition((int32_t)motorPosition);
		String message = "motorPosition set to " + String((int32_t)motorPosition);
		rosNodeHandler.loginfo(string2char(message));
	}
	else
	{
		rosNodeHandler.loginfo("motorPosition not set. Emergency Stop state.");
		logSD("ERROR: motorPosition not set. Emergency Stop state.");
	}
}

void motorOperationState_cb(const std_msgs::Bool& cmd_msg)
{
	logSD("Motor Operation State : Retrieving");
	publishMotorCurrentOperationState();
}

void lidarScan_cb(const std_msgs::Bool& cmd_msg)
{
	lidarScanMode = cmd_msg.data;
	rosNodeHandler.loginfo("LIDAR Scan Mode: " + (lidarScanMode) ? "On" : "Off");
	logSD("LIDAR Scan Mode: " + (lidarScanMode) ? "On" : "Off");
}

void musicLights_cb(const std_msgs::Bool& cmd_msg)
{
	musicLightsMode = cmd_msg.data;
	rosNodeHandler.loginfo("Music Lights: " + (musicLightsMode) ? "On" : "Off");
	logSD("Music Lights: " + (musicLightsMode) ? "On" : "Off");
}

void ackermann_cb(const ackermann_msgs::AckermannDriveStamped& cmd_msg)
{
	// Save to global variables
	speed = cmd_msg.drive.speed;
	acceleration = cmd_msg.drive.acceleration;
	jerk = cmd_msg.drive.jerk;
	steering_angle = cmd_msg.drive.steering_angle;
	steering_angle_velocity = cmd_msg.drive.steering_angle_velocity;

	// Convert to std_msg to use in local function
	std_msgs::Float32 std_msg_speed;
	std_msgs::Float32 std_msg_steering_angle;

	// Use /speed function to set speed
	std_msg_speed.data = speed;
	motorSpeed_cb(std_msg_speed);
	logSD("Ackermann Drive_Speed set to " + String(std_msg_speed.data));

	// Use /steering function to set steering
	std_msg_steering_angle.data = cmd_msg.drive.steering_angle;
	steering_cb(std_msg_steering_angle);
	logSD("Ackermann Drive_Steering_Angle set to " + String(std_msg_steering_angle.data));
}

//#################################################################################################
// RoS subscriber routine
//#################################################################################################

// Register RoS Subsriber Handlers
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> subAckermann(ackermann_id, ackermann_cb);
ros::Subscriber<std_msgs::Int16> subHeadTilt(headTilt_id, headTilt_cb);
ros::Subscriber<std_msgs::Int16> subHeadPan(headPan_id, headPan_cb);
ros::Subscriber<std_msgs::Float32> subMotorSpeed(motorSpeed_id, motorSpeed_cb);
ros::Subscriber<std_msgs::Float32> subMotorPosition(motorPosition_id, motorPosition_cb);
ros::Subscriber<std_msgs::Bool>  subMotorEmergencyStop(motorEmergencyStopSetting_id, motorEmergencyStop_cb);
ros::Subscriber<std_msgs::Bool>  subMotorEnergise(motorEnergise_id, motorEnergise_cb);
ros::Subscriber<std_msgs::Float32> subSteering(steering_id, steering_cb);
ros::Subscriber<std_msgs::Bool>  subMotorOperationState(motorOperationState_id, motorOperationState_cb);
ros::Subscriber<std_msgs::Bool>  subMusicLights(musicLights_id, musicLights_cb);
ros::Subscriber<std_msgs::Bool>  subLidarScan(lidar_scan_enable_id, lidarScan_cb);

//#################################################################################################
// Timer Loops
//#################################################################################################

void imuLoop()
{
	// Body IMU
	// If intPin goes high, all data registers have new data
	// On interrupt, check if data ready interrupt
	// Note ROS and IMU orientation are different
	/*********************************************
	IMU Orientation
		x forward
		y right
		z up

	ROS Axis Orientation
	================
		x forward
		y left
		z up

		X east
		Y north
		Z up
	*********************************************/
	if (bodyImu.readByte(MPU9250_ADDRESS_AD0, INT_STATUS) & 0x01)
	{
		// Load the body values
		bodyImu.readAccelData(bodyImu.accelCount);  // Read the x/y/z adc values

		bodyImu.ax = (float)bodyImu.accelCount[0] * bodyImu.aRes;
		bodyImu.ay = -(float)bodyImu.accelCount[1] * bodyImu.aRes;
		bodyImu.az = (float)bodyImu.accelCount[2] * bodyImu.aRes;

		bodyImu.readGyroData(bodyImu.gyroCount);  // Read the x/y/z adc values

		bodyImu.gx = (float)bodyImu.gyroCount[0] * bodyImu.gRes;
		bodyImu.gy = -(float)bodyImu.gyroCount[1] * bodyImu.gRes;
		bodyImu.gz = (float)bodyImu.gyroCount[2] * bodyImu.gRes;

		bodyImu.readMagData(bodyImu.magCount);

		bodyImu.mx = (float)bodyImu.magCount[0] * bodyImu.mRes;
		bodyImu.my = -(float)bodyImu.magCount[1] * bodyImu.mRes;
		bodyImu.mz = (float)bodyImu.magCount[2] * bodyImu.mRes;

		bodyImu.updateTime();

		bodyImuFilter.MadgwickQuaternionUpdate(bodyImu.ax, bodyImu.ay, bodyImu.az, 
			bodyImu.gx * DEG_TO_RAD, bodyImu.gy * DEG_TO_RAD, bodyImu.gz * DEG_TO_RAD, 
			bodyImu.mx, bodyImu.my, bodyImu.mz, bodyImu.deltat);

		bodyImu.count = millis();
		bodyImu.sumCount = 0;
		bodyImu.sum = 0;
	}
}

void lidarLoop()
{
	if (frontLidar.available())
	{
		float distance = frontLidar.getDistance();
		float strength = frontLidar.getStrength();

		if (distance >= LIDAR_MIN_RANGE*100 && distance < LIDAR_MAX_RANGE*100)
		{
			lidarRange = distance; // Distance in cm
			lidarIntensity = frontLidar.getStrength();
		}
		lidarUpdateTime = micros();
	}

	if (lidarScanMode)
	{
		switch (lidarScanState)
		{
		case LidarScanState::Start:
			// Move head to the left and level
			servoMove(HEAD_PAN, -100);
			servoMove(HEAD_TILT, 0);
			headTiltRadians = 0;
			headPanRadians = 0.6981; // 100% left in rad
			lidar_counter = 0;
			lidarInitialiseWait = millis() + 1000;
			lidarScanState = LidarScanState::Initialising;
			
			break;

		case LidarScanState::Initialising:
			// Wait for 1 second
			if (millis() >= lidarInitialiseWait)
			{
				lidarScanState = LidarScanState::BeginScanning;
			}

			break;

		case LidarScanState::BeginScanning:
			temp_lidar_scan_msg.header.frame_id = lidar_scan_id;
			temp_lidar_scan_msg.header.stamp = rosNodeHandler.now();

			temp_lidar_scan_msg.angle_increment = ((float)HEAD_PAN_RANGE * 2.0) / (float)LIDAR_HEAD_PAN_STEPS * 0.0174533; // radians per step;
			temp_lidar_scan_msg.angle_max = (float)HEAD_PAN_RANGE * 0.0174533; // convert deg to rad
			temp_lidar_scan_msg.angle_min = -(float)HEAD_PAN_RANGE * 0.0174533; // convert deg to rad

			temp_lidar_scan_msg.ranges_length = LIDAR_HEAD_PAN_STEPS;
			temp_lidar_scan_msg.intensities_length = LIDAR_HEAD_PAN_STEPS;

			temp_lidar_scan_msg.range_min = LIDAR_MIN_RANGE;
			temp_lidar_scan_msg.range_max = LIDAR_MAX_RANGE;

			temp_lidar_scan_msg.ranges = new float[LIDAR_HEAD_PAN_STEPS];
			temp_lidar_scan_msg.intensities = new float[LIDAR_HEAD_PAN_STEPS];
			
			addScanData();

			lidarScanState = LidarScanState::Scanning;

			break;

		case LidarScanState::Scanning:
			addScanData();

			if (lidar_counter == LIDAR_HEAD_PAN_STEPS)
				lidarScanState = LidarScanState::EndScanning;

			break;

		case LidarScanState::EndScanning:
			lidar_scan_msg = temp_lidar_scan_msg; // Copy over to send
			temp_lidar_scan_msg = *(new sensor_msgs::LaserScan()); // Create a new instance for saving scans
			temp_lidar_scan_msg.scan_time = (float)(rosNodeHandler.now().toNsec() - lidar_scan_msg.header.stamp.toNsec()) / 1000000000.0;
			temp_lidar_scan_msg.time_increment = temp_lidar_scan_msg.scan_time / (float)LIDAR_HEAD_PAN_STEPS;
			lidarScanState = LidarScanState::Start;
			sendLidarMsg = true;
			lidarScanState = LidarScanState::Start;
			
			break;

		default:
			break;
		}
	}
	else
	{
		lidarScanState = LidarScanState::Start;
	}
}

void addScanData()
{
	// The range data must be <12 meters, >= 0.3 meters and stregth > 20% (20) for it to be valid
	if (lidarRange >= LIDAR_MIN_RANGE * 100.0 && lidarRange < LIDAR_MAX_RANGE * 100.0)
	{
		temp_lidar_scan_msg.ranges[lidar_counter] = lidarRange / 100; // convert to meters
		temp_lidar_scan_msg.intensities[lidar_counter] = lidarIntensity;
	}
	else
	{
		temp_lidar_scan_msg.ranges[lidar_counter] = 0;
		temp_lidar_scan_msg.intensities[lidar_counter] = 0;
	}

	lidar_counter++;

	scan_angle = -100 + (LIDAR_HEAD_PAN_STEP * lidar_counter);
	servoMove(HEAD_PAN, scan_angle);
	headPanRadians = scan_angle * -0.006981; // convert % to rad

	if (lidar_counter == LIDAR_HEAD_PAN_STEPS)
		lidarScanState = LidarScanState::EndScanning;
}

void rangeLoop()
{
	uint32_t start = millis();

	while (Serial5.available() && millis() - start <= 10)
	{
		String response = Serial5.readStringUntil(10);

		StaticJsonDocument<200> data;

		// Deserialize the JSON document
		DeserializationError error = deserializeJson(data, response);

		if (!error)
		{
			// We have a request for data
			frontUltrasonicRange = data["front_ultrasonic"];
			rearUltrasonicRange = data["rear_ultrasonic"];

			frontUltrasonicTimestamp = millis();
			rearUltrasonicTimestamp = millis();
		}
	}

	// Check if we're too close to the front and moving forwards
	if ((motorCurrentSpeedVal > 0 && frontUltrasonicRange < BRAKE_DISTANCE) || (motorCurrentSpeedVal < 0 && rearUltrasonicRange < BRAKE_DISTANCE))
	{
		if (!tooCloseSet)
		{
			tooCloseSet = true;
			previousSpeed = speed;
			speed = 0;
			std_msgs::Float32 cmd_msg;
			cmd_msg.data = 0;
			motorSpeed_cb(cmd_msg);
		}
	}
	else if ((previousSpeed > 0 && frontUltrasonicRange >= BRAKE_DISTANCE) || (previousSpeed < 0 && rearUltrasonicRange >= BRAKE_DISTANCE))
	{
		if (tooCloseSet)
		{
			speed = previousSpeed;
			previousSpeed = 0;
			std_msgs::Float32 cmd_msg;
			cmd_msg.data = speed;
			motorSpeed_cb(cmd_msg);
			tooCloseSet = false;
		}
	}

	if (millis() - start > 30)
	{
		logSD("Serial5 (Ultrasonic) timed out.");
	}
}

// Run every 100mS (10Hz)
void publishRosDataLoop()
{
	// Flash the LED to let us know it's updating
	ledState = !ledState;
	digitalWrite(LED_BUILTIN, ledState);

	if (rosNodeHandler.connected())
	{
		// Get the front ultrasonic data.
		publishFrontUltrasonicRange();

		// Get the rear ultrasonic data.
		publishRearUltrasonicRange();
		
		// Get the lidar data.
		publishLidar();

		// Get the lidar scan data.
		publishLidarScan();

		// Publish the head joint position
		publishJointState();

		// Publish the Head IMU
		publishBodyImu();

		// Publish the motor speed
		publishMotorCurrentSpeed();
		
		// Publish the motor position
		publishMotorCurrentPosition();

		// Publish the odometry
		publishOdometry();

		// Check the motor
		if (tic.getErrorStatus() & (uint16_t)TicError::MotorDriverError && !motorEmergencyStopVal)
		{
			rosNodeHandler.logerror("Motor driver error");
		}

		// Check the motor voltage
		if (tic.getErrorStatus() & (uint16_t)TicError::LowVin)
		{
			rosNodeHandler.logerror("Motor driver low voltage error");
		}
	}
}
//#################################################################################################
// Setup
//#################################################################################################

void setup()
{

	// Set up the pins
	pinMode(FRONT_ULTRASONIC_TRIG_PIN, OUTPUT);
	pinMode(FRONT_ULTRASONIC_ECHO_PIN, INPUT);
	pinMode(REAR_ULTRASONIC_TRIG_PIN, OUTPUT);
	pinMode(REAR_ULTRASONIC_ECHO_PIN, INPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(SERIAL5_RX_PIN, INPUT_PULLDOWN);

	//AudioMemory(12);
	// Removed above due to issues in Teensy Audio libery
	AudioStream::initialize_memory(audioMemoryBlock, MEMORY_BLOCK_SIZE);

	fft1.averageTogether(8);

	digitalWrite(LED_BUILTIN, 1);

	// Set up the Mouth LEDs
	FastLED.addLeds<WS2812B,MOUTH_LED_DATA_PIN, GRB>(mouthLeds, MOUTH_NUM_LEDS, 0);

	// Set up the Pi LEDs
	FastLED.addLeds<WS2812B, PI_LED_DATA_PIN, GRB>(piLeds, PI_NUM_LEDS);

	// Set up the Pi LEDs
	FastLED.addLeds<WS2812B, ODROID_LED_DATA_PIN, GRB>(odroidLeds, ODROID_NUM_LEDS);

	piLeds[0] = CRGB::Green;

	FastLED.show();
	
	// see if the card is present and can be initialized:
	if (SD.begin(BUILTIN_SDCARD))
	{
		int filenum = 0;

		filename = "log_" + String(filenum) + ".txt";

		// Loop until we find a fiename that doesn't exist
		while (SD.exists(filename.c_str()) && filenum < 10000)
		{
			filename = "log_" + String(filenum) + ".txt";
			filenum++;
		}

		if (filenum >= 10000)
		{
			// Too many logs so don't log
			logSDEnable = false;
		}

		piLeds[1] = CRGB::Green;
		FastLED.show();
	}
	else
	{
		// No card so don't log
		logSDEnable = false;

		piLeds[1] = CRGB::Red;
		FastLED.show();
	}

	logSD("Starting Setup");

	logSD("Initialising TFmini");
	Serial1.setRX(SERIAL1_RX_PIN);
	Serial1.setTX(SERIAL1_TX_PIN);
	Serial1.begin(TFmini::DEFAULT_BAUDRATE);

	// Wait for up to 3 seconds to connect
	uint32_t waitTime = millis() + 3000;

	bool ledOnOff = false;
	while (!Serial1 && millis() < waitTime)
	{
		// Flash the status LED amber
		if (ledOnOff)
		{
			piLeds[2] = CRGB::Yellow;
			FastLED.show();
			ledOnOff = false;
		}
		else
		{
			piLeds[2] = CRGB::Black;
			FastLED.show();
			ledOnOff = true;
		}
		delay(100);
	}

	// Wait for up to 3 seconds to connect
	waitTime = millis() + 3000;


	logSD("Initialise ESP32 Serial5");
	Serial5.setRX(SERIAL5_RX_PIN);
	Serial5.setTX(SERIAL5_TX_PIN);
	Serial5.begin(SERIAL5_BAUD_RATE);

	while (!Serial5 && millis() < waitTime)
	{
		// Flash the status LED amber
		if (ledOnOff)
		{
			piLeds[2] = CRGB::Purple;
			FastLED.show();
			ledOnOff = false;
		}
		else
		{
			piLeds[2] = CRGB::Black;
			FastLED.show();
			ledOnOff = true;
		}
		delay(100);
	}

	// If all the rangefinder serial connections connected
	if (Serial1 && Serial5)
	{
		lidarActive = true;
		frontLidar.attach(Serial1);

		piLeds[2] = CRGB::Green;
		FastLED.show();
	}
	else
	{
		piLeds[2] = CRGB::Red;
		FastLED.show();
	}

	logSD("Wire.begin");
	// Motor and Servos
	Wire.setSDA(I2C0_SDA_PIN);
	Wire.setSCL(I2C0_SCL_PIN);
	Wire.begin();
	
	// Body IMU
	Wire2.setSDA(I2C2_SDA_PIN);
	Wire2.setSCL(I2C2_SCL_PIN);
	Wire2.begin();

	delay(500);

	bool wireError = false;
	// Read the Servo controller response
	Wire.beginTransmission(0x40);
	
	if (Wire.endTransmission() == 0)
	{
		logSD("Servo controller found at address 0x40");
	}
	else
	{
		logSD("ERROR: Servo controller NOT found at address 0x40");
		wireError = true;
	}

	// ##############################################################
	// Read the Body IMU
	byte imuWho = bodyImu.readByte(MPU9250_ADDRESS_AD0, WHO_AM_I_MPU9250);

	if (imuWho == 0x71)
	{
		logSD("IMU Body OK. Responded to who with "+String(imuWho,HEX));
	}
	else
	{
		logSD("ERROR: Body IMU NOT found. Responded to who with " + String(imuWho, HEX));
		wireError = true;
	}

	tic.haltAndSetPosition(0);

	// Display red LED if any of the SPI tests failed
	if (!wireError)
	{
		piLeds[3] = CRGB::Green;
		FastLED.show();
	}
	else
	{
		piLeds[3] = CRGB::Red;
		FastLED.show();
	}
	
	// Calibrate the IMU's
	logSD("Calibrating Body IMU");
	bodyImu.calibrateMPU9250(bodyImu.gyroBias, bodyImu.accelBias);

	logSD("Calibrating Body IMU Magnetometer");
	// Get magnetometer calibration from AK8963 ROM
	bodyImu.initAK8963(bodyImu.factoryMagCalibration);

	// Get sensor resolutions, only need to do this once
	bodyImu.getAres();
	bodyImu.getGres();
	bodyImu.getMres();

	piLeds[4] = CRGB::YellowGreen;
	FastLED.show();

	logSD("servos.begin");
	servos.begin();
	servos.setPWMFreq(60); // Analog servo frequency

	// initialize variables to pace updates to correct rate
	microsPerReading = 1000000 / 25;
	microsPrevious = micros();

	logSD("Initialise Servos");
	servoMove(STEERING, 0);
	servoMove(HEAD_PAN, 0);
	servoMove(HEAD_TILT, 0);

	tic.setStartingSpeed(0);
	tic.clearDriverError();
	tic.energize();
	tic.exitSafeStart();
	

	piLeds[4] = CRGB::Green;
	FastLED.show();

	logSD("Initialise Node Handler");

	logSD("Initialise Node Handler and Transform Broadcaster");

	rosNodeHandler.initNode();
	transformBroadcaster.init(rosNodeHandler);

	logSD("Initialise Node Handler Subscribers");
	rosNodeHandler.subscribe(subHeadTilt);
	rosNodeHandler.subscribe(subHeadPan);
	rosNodeHandler.subscribe(subMotorSpeed);
	rosNodeHandler.subscribe(subMotorPosition);
	rosNodeHandler.subscribe(subMotorEmergencyStop);
	rosNodeHandler.subscribe(subMotorEnergise);
	rosNodeHandler.subscribe(subSteering);
	rosNodeHandler.subscribe(subMotorOperationState);
	rosNodeHandler.subscribe(subMusicLights);
	rosNodeHandler.subscribe(subLidarScan);
	rosNodeHandler.subscribe(subAckermann);

	logSD("Initialise Node Handler Publishers");
	rosNodeHandler.advertise(pubLidar);
	rosNodeHandler.advertise(pubLidarScan);
	rosNodeHandler.advertise(pubUltrasonicFront);
	rosNodeHandler.advertise(pubUltrasonicRear);
	rosNodeHandler.advertise(pubMotorCurrentSpeed);
	rosNodeHandler.advertise(pubMotorCurrentPosition);
	rosNodeHandler.advertise(pubMotorCurrentOperationState);
	rosNodeHandler.advertise(pubBodyImu);
	rosNodeHandler.advertise(pubOdometry);
	rosNodeHandler.advertise(pubJointState);

	piLeds[5] = CRGB::Green;
	FastLED.show();

	// Wait two seconds so we can see the LED status
	delay(2000);

	// Set up the process threads
	// ##########################

	logSD("Starting IMU Loop");

	// IMU Timer is 100Hz
	imuTimer.priority(120);
	imuTimer.begin(imuLoop, 1000000 / IMU_LOOP_HZ);

	logSD("Starting LIDAR and Ultrasonic Loop");

	// Set up the odemetry time
	odom_current_millis = millis();
	odom_last_millis = millis();

	logSD("Setup Conplete");
}

//#################################################################################################
// Process Loop
//#################################################################################################

void loop()
{	
	uint32_t finish = millis() + (1000 / MAIN_LOOP_HZ);

	lidarLoop();
	rangeLoop();
	publishRosDataLoop();
	processLEDLoop();
	spinResetCommandTimeout();
	
	while (millis() < finish)
	{
		spinResetCommandTimeout();
	}
}
