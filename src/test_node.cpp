#include "ros/ros.h"
#include <math.h>


#include "std_msgs/Float64MultiArray.h"
#include "ressac_msgs/UavState.h"
#include <visioland_observers/observer_base.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

double roll=0, pitch=0, yaw=0;
Vector4d y_v=Vector4d::Zero();
Vector3d v=Vector3d::Zero();

MatrixXd P(5,5);

VectorXd hat_z(5);

double te=1.00/30.00;

//Introduce a virtual cam made by the sensor fusion
double X_v1=0,X_v2=0,Y_v1=0,Y_v2=0,Z_v=0;

//Test between speed and derivate of the position
double X_v1_old=0,Y_v1_old=0,Z_v_old=0;
Vector3d v_test=Vector3d::Zero();

ros::Publisher state_publisher;

EKF_Observer Observer;

//Usefull to display testing measure or virtual cam measure
Vector4d y_v_test;

Matrix3d R_c_q;

bool data_new;
Vector4d y;

void chatterCallCamera(std_msgs::Float64MultiArray msg){
  data_new=true;

  y[0]=msg.data[0];// measure from the left intersection
  y[1]=msg.data[1];
  y[2]=msg.data[2];// measure from the right intersection
  y[3]=msg.data[3];

}

//Get the orientation and the speed from the ssf
void chatterCall(ressac_msgs::UavState uav_state){
	
	R_c_q=AngleAxisd(uav_state.attitude.z,Vector3d::UnitZ())*AngleAxisd(uav_state.attitude.y,Vector3d::UnitY())*AngleAxisd(uav_state.attitude.x,Vector3d::UnitX());
	//std::cout<<"Rc "<<R_c_q<<std::endl;

	v<<uav_state.velocity.x,uav_state.velocity.y,uav_state.velocity.z;
	//std::cout<<"Speed "<<v<<std::endl;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "observer");
  ros::NodeHandle n;
  ros::Subscriber sub_uavState=n.subscribe("ressac_reader/uav_state",1000,chatterCall);
  ros::Subscriber sub = n.subscribe("visual_landing/tracking",1000,chatterCallCamera);

  state_publisher = n.advertise<std_msgs::Float64MultiArray>("visual_landing/observer_state", 1000);

  Observer.start();
  data_new=false;


  ros::Rate loop_rate(30);

  while (ros::ok()){

      VectorXd hat_z(5);
      if(data_new){
        data_new=false;
        hat_z=Observer.update(R_c_q,y,v);
        //Create a msg in order to publish the state
        std_msgs::Float64MultiArray state_msg;

        // This message is used for debug purpose 
        state_msg.data.clear();
        state_msg.data.push_back(hat_z(0)); //y_v_hat_1      0
        state_msg.data.push_back(hat_z(1)); //y_v_hat_2
        state_msg.data.push_back(hat_z(2)); //y_v_hat_1
        state_msg.data.push_back(hat_z(3)); //y_v_hat_2
        state_msg.data.push_back(hat_z(4)); //eta_hat        4

        state_msg.data.push_back(hat_z(0)/hat_z(4)); //X_v   5
        state_msg.data.push_back(hat_z(1)/hat_z(4)); //y_v
        state_msg.data.push_back(1/hat_z(4)); //Z_v          7

        state_msg.data.push_back(y_v(0)); //y_v              8
        state_msg.data.push_back(y_v(1)); //y_v
        state_msg.data.push_back(y_v(2)); //y_v
        state_msg.data.push_back(y_v(3)); //y_v              11

        state_msg.data.push_back(1/Z_v); //eta               12

        state_msg.data.push_back(X_v1);                    //13
        state_msg.data.push_back(Y_v1);
        state_msg.data.push_back(Z_v);                     //15


        y_v_test<<X_v1/Z_v,Y_v1/Z_v,X_v2/Z_v,Y_v2/Z_v;

        state_msg.data.push_back(y_v_test(0));             //16 Virtual cam measure by the ptam or Measure without stabilization
        state_msg.data.push_back(y_v_test(1));
        state_msg.data.push_back(y_v_test(2));
        state_msg.data.push_back(y_v_test(3));             //19

        state_publisher.publish(state_msg);

      }

      ros::spinOnce();

      loop_rate.sleep();

  }

  return 0;
}
