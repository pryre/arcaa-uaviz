#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

#include <string>
#include <math.h>


geometry_msgs::Quaternion toQuaternion(geometry_msgs::Vector3 e) {
    geometry_msgs::Quaternion q;

    double t0 = cos(e.z * 0.5);
    double t1 = sin(e.z * 0.5);
    double t2 = cos(e.x * 0.5);
    double t3 = sin(e.x * 0.5);
    double t4 = cos(e.y * 0.5);
    double t5 = sin(e.y * 0.5);

    q.w = t2 * t4 * t0 + t3 * t5 * t1;
    q.x = t3 * t4 * t0 - t2 * t5 * t1;
    q.y = t2 * t5 * t0 + t3 * t4 * t1;
    q.z = t2 * t4 * t1 - t3 * t5 * t0;

	return q;
}

int main( int argc, char **argv ) {
	ros::init( argc, argv, "uaviz" );
	ros::NodeHandle nh( ros::this_node::getName() );

	std::string param_vicon_frame = "world";
	std::string param_uav_frame = "uav";
	double param_pub_rate = 1.0;
	std::string param_uav_type = "X4";
	double param_uav_scale = 0.25;	//0.5m span
	double param_uav_rotor_scale = 0.2;

	ros::Rate loop_rate( param_pub_rate );

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>( "/visualization/marker", 0 );
	ros::Publisher marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>( "/visualization/marker_array", 0 );

	//== Floor Marker
	visualization_msgs::Marker marker_floor;
	//Header
	marker_floor.header.frame_id = param_vicon_frame;
	//Marker
	marker_floor.ns = "hangar_room";
	marker_floor.id = 0;
	marker_floor.type = visualization_msgs::Marker::CUBE;
	marker_floor.action = visualization_msgs::Marker::ADD;
	marker_floor.lifetime = ros::Duration(0.0);
	marker_floor.frame_locked = true;
	//Pose
	marker_floor.pose.position.x = 0.0;
	marker_floor.pose.position.y = 0.0;
	marker_floor.pose.position.z = -0.05;
	marker_floor.pose.orientation.x = 0.0;
	marker_floor.pose.orientation.y = 0.0;
	marker_floor.pose.orientation.z = 0.0;
	marker_floor.pose.orientation.w = 1.0;
	//Scale
	marker_floor.scale.x = 4.2;
	marker_floor.scale.y = 4.2;
	marker_floor.scale.z = 0.1;
	//Color
	marker_floor.color.a = 1.0;
	marker_floor.color.r = 0.5;
	marker_floor.color.g = 0.5;
	marker_floor.color.b = 0.5;


	//== Wall Markers
	visualization_msgs::Marker marker_wall_a;
	visualization_msgs::Marker marker_wall_b;
	visualization_msgs::Marker marker_wall_c;
	visualization_msgs::Marker marker_wall_d;
	//Header
	marker_wall_a.header.frame_id = param_vicon_frame;
	//Marker
	marker_wall_a.ns = "hangar_room";
	marker_wall_a.id = 1;
	marker_wall_a.type = visualization_msgs::Marker::CUBE;
	marker_wall_a.action = visualization_msgs::Marker::ADD;
	marker_wall_a.lifetime = ros::Duration( 0.0 );
	marker_wall_a.frame_locked = true;
	//Pose
	marker_wall_a.pose.position.x = 2.05;
	marker_wall_a.pose.position.y = 0.0;
	marker_wall_a.pose.position.z = 1.0;
	marker_wall_a.pose.orientation.x = 0.0;
	marker_wall_a.pose.orientation.y = 0.0;
	marker_wall_a.pose.orientation.z = 0.0;
	marker_wall_a.pose.orientation.w = 1.0;
	//Scale
	marker_wall_a.scale.x = 0.1;
	marker_wall_a.scale.y = 4.0;
	marker_wall_a.scale.z = 2.0;
	//Color
	marker_wall_a.color.a = 1.0;
	marker_wall_a.color.r = 0.8;
	marker_wall_a.color.g = 0.2;
	marker_wall_a.color.b = 0.2;
	//Repeat B
	marker_wall_b = marker_wall_a;	//Copy wall A
	marker_wall_b.id = 2;
	marker_wall_b.pose.position.x = 0.0;
	marker_wall_b.pose.position.y = 2.05;
	marker_wall_b.scale.x = 4.0;
	marker_wall_b.scale.y = 0.1;
	//Repeat C
	marker_wall_c = marker_wall_a;	//Copy wall A
	marker_wall_c.id = 3;
	marker_wall_c.pose.position.x = -2.05;
	//Repeat D
	marker_wall_d = marker_wall_b;	//Copy wall B
	marker_wall_d.id = 4;
	marker_wall_d.pose.position.y = -2.05;


	//== UAV Markers
	visualization_msgs::Marker marker_uav_frame;
	visualization_msgs::MarkerArray marker_uav_arms;
	visualization_msgs::MarkerArray marker_uav_rotors;
	//Frame
	//Header
	marker_uav_frame.header.frame_id = param_uav_frame;
	//Marker
	marker_uav_frame.ns = "uav_frame";
	marker_uav_frame.id = 0;
	marker_uav_frame.type = visualization_msgs::Marker::CYLINDER;
	marker_uav_frame.action = visualization_msgs::Marker::ADD;
	marker_uav_frame.lifetime = ros::Duration( 0.0 );
	marker_uav_frame.frame_locked = true;
	//Pose
	marker_uav_frame.pose.position.x = 0.0;
	marker_uav_frame.pose.position.y = 0.0;
	marker_uav_frame.pose.position.z = 0.0;
	marker_uav_frame.pose.orientation.x = 0.0;
	marker_uav_frame.pose.orientation.y = 0.0;
	marker_uav_frame.pose.orientation.z = 0.0;
	marker_uav_frame.pose.orientation.w = 1.0;
	//Scale
	marker_uav_frame.scale.x = param_uav_rotor_scale;
	marker_uav_frame.scale.y = param_uav_rotor_scale;
	marker_uav_frame.scale.z = 0.05;
	//Color
	marker_uav_frame.color.a = 1.0;
	marker_uav_frame.color.r = 1.0;
	marker_uav_frame.color.g = 1.0;
	marker_uav_frame.color.b = 1.0;


	if ( param_uav_type.compare( "X4" ) == 0 ) {
		double step_size = 2 * M_PI / 4;

		//Arms
		for( int i = 0; i < 4; i++ ) {
			//Rotate an X unit vector by rot, scaled by the arm length
			double rot = ( i * step_size ) + (step_size / 2.0);
			double pox_x = ( param_uav_scale / 2.0 ) * std::cos( rot ); //No Y axis element in a X unit vector
			double pox_y = ( param_uav_scale / 2.0 ) * std::sin( rot );

			visualization_msgs::Marker arm;
			//Header
			arm.header.frame_id = param_uav_frame;
			//Marker
			arm.ns = "uav_arms";
			arm.id = i;
			arm.type = visualization_msgs::Marker::CYLINDER;
			arm.action = visualization_msgs::Marker::ADD;
			arm.lifetime = ros::Duration(0.0);
			arm.frame_locked = true;
			//Pose
			arm.pose.position.x = pox_x;
			arm.pose.position.y = pox_y;
			arm.pose.position.z = 0.075;
			geometry_msgs::Vector3 armHeading;
			armHeading.z = rot;
			arm.pose.orientation = toQuaternion(armHeading);
			//Scale
			arm.scale.x = 0.05;
			arm.scale.y = 0.05;
			arm.scale.z = param_uav_scale;
			//Color
			arm.color.a = 0.5;
			arm.color.r = 0.8;
			arm.color.g = 0.8;
			arm.color.b = 0.8;

			//Add arm to the array
			marker_uav_arms.markers.push_back(arm);
		}

		//Rotors
		for( int i = 0; i < 4; i++ ) {
			//Rotate an X unit vector by rot, scaled by the arm length
			double rot = ( i * step_size ) + (step_size / 2.0);
			double pox_x = param_uav_scale * std::cos( rot ); //No Y axis element in a X unit vector
			double pox_y = param_uav_scale * std::sin( rot );

			visualization_msgs::Marker rotor;
			//Header
			rotor.header.frame_id = param_uav_frame;
			//Marker
			rotor.ns = "uav_rotors";
			rotor.id = i;
			rotor.type = visualization_msgs::Marker::CYLINDER;
			rotor.action = visualization_msgs::Marker::ADD;
			rotor.lifetime = ros::Duration( 0.0 );
			rotor.frame_locked = true;
			//Pose
			rotor.pose.position.x = pox_x;
			rotor.pose.position.y = pox_y;
			rotor.pose.position.z = 0.03;
			rotor.pose.orientation.x = 0.0;
			rotor.pose.orientation.y = 0.0;
			rotor.pose.orientation.z = 0.0;
			rotor.pose.orientation.w = 1.0;
			//Scale
			rotor.scale.x = param_uav_rotor_scale;
			rotor.scale.y = param_uav_rotor_scale;
			rotor.scale.z = 0.01;
			//Color
			rotor.color.a = 0.5;
			rotor.color.r = 0.8;
			rotor.color.g = 0.8;
			rotor.color.b = 0.8;

			//Add rotor to the array
			marker_uav_rotors.markers.push_back( rotor );
		}

	} else {
		ROS_ERROR( "Unsupported frame type: %s", param_uav_type.c_str() );
		ros::shutdown();
	}

	//TODO: UAV Path


	while ( ros::ok() ) {
		ros::Time timestamp = ros::Time::now();

		//== Room Markers
		marker_floor.header.stamp = timestamp;
		marker_floor.header.seq++;
		marker_wall_a.header.stamp = timestamp;
		marker_wall_a.header.seq++;
		marker_wall_b.header.stamp = timestamp;
		marker_wall_b.header.seq++;
		marker_wall_c.header.stamp = timestamp;
		marker_wall_c.header.seq++;
		marker_wall_d.header.stamp = timestamp;
		marker_wall_d.header.seq++;

		//== UAV Markers
		marker_uav_frame.header.stamp = timestamp;
		marker_uav_frame.header.seq++;

		/*TODO:
		for( int i = 0; i < marker_uav_arms.markers.size(); i++ ) {
			marker_uav_arms.markers.at(i).header.stamp = timestamp;
			marker_uav_arms.markers.at(i).header.seq++;
		}
		*/

		for( int i = 0; i < marker_uav_rotors.markers.size(); i++ ) {
			marker_uav_rotors.markers.at(i).header.stamp = timestamp;
			marker_uav_rotors.markers.at(i).header.seq++;
		}

		//== Publish
		marker_pub.publish( marker_floor );
		marker_pub.publish( marker_wall_a );
		marker_pub.publish( marker_wall_b );
		marker_pub.publish( marker_wall_c );
		marker_pub.publish( marker_wall_d );

		marker_pub.publish( marker_uav_frame );
		//marker_array_pub.publish( marker_uav_arms );
		marker_array_pub.publish( marker_uav_rotors );

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
