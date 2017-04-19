#include <visioland_observers/observer_base.h>
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;


EKF_Observer::EKF_Observer() : P(5,5),hat_z(5)
{}

void EKF_Observer::start(){

  P<<(0.4*0.2/3)*(0.4*0.2/3),0,0,0,0, 0,(0.6*0.2/3)*(0.6*0.2/3),0,0,0, 0,0,(0.4*0.2/3)*(0.4*0.2/3),0,0,0,0,0,(0.6*0.2/3)*(0.6*0.2/3),0,0,0,0,0,(0.4*0.2/3)*(0.4*0.2/3);
  hat_z<<10.8715,-0.436539,10.9176,-0.22682,0.1;

}


VectorXd EKF_Observer::update(Matrix3d R_c_q,Vector4d y,Vector3d v){
      Vector4d y_v;

      Matrix3d R_c;


      Matrix3d R_c_0;

      R_c_0<<0,0,1,1,0,0,0,1,0;

      R_c=R_c_0*R_c_q;

      //std::cout<<"R_c \n"<<R_c<<std::endl;

      y_v<<(R_c(0,0)*y[0]+R_c(0,1)*y[1]+R_c(0,2))/(R_c(2,0)*y[0]+R_c(2,1)*y[1]+R_c(2,2)),(R_c(1,0)*y[0]+R_c(1,1)*y[1]+R_c(1,2))/(R_c(2,0)*y[0]+R_c(2,1)*y[1]+R_c(2,2)),
           (R_c(0,0)*y[2]+R_c(0,1)*y[3]+R_c(0,2))/(R_c(2,0)*y[2]+R_c(2,1)*y[3]+R_c(2,2)),(R_c(1,0)*y[2]+R_c(1,1)*y[3]+R_c(1,2))/(R_c(2,0)*y[2]+R_c(2,1)*y[3]+R_c(2,2));


    double te=1.0/30.0;
    double eta=hat_z[4];

    MatrixXd H(4,5);

    H<<1,0,0,0,0,
       0,1,0,0,0,
       0,0,1,0,0,
       0,0,0,1,0;

    VectorXd g(5);

    g<<(v[2]*y_v[0]-v[0])*eta,
         (v[2]*y_v[1]-v[1])*eta,
         (v[2]*y_v[2]-v[0])*eta,
         (v[2]*y_v[3]-v[1])*eta,
         v[2]*eta*eta;


    MatrixXd F(5,5);

    F<<eta*v[2],        0,        0,        0, v[2]*y_v[0] - v[0],
              0, eta*v[2],        0,        0, v[2]*y_v[1] - v[1],
              0,        0, eta*v[2],        0, v[2]*y_v[2] - v[0],
              0,        0,        0, eta*v[2], v[2]*y_v[3] - v[1],
              0,        0,        0,        0,         2*eta*v[2];


    Matrix4d R_inv;

    double k1=1.0/100.0,k2=1.0/0.010; //k1 on the y1 measure, k2 on the y2 measure


    R_inv<<k1,0,0,0,
           0,k2,0,0,
           0,0,k1,0,
           0,0,0,k2;

    MatrixXd Q(5,5);

    Q<<MatrixXd::Identity(5,5)*10;

    MatrixXd K(5,4);

    K=P*H.transpose()*R_inv;


    MatrixXd dot_P(5,5);

    dot_P=F*P+P*F.transpose()-K*H*P+Q;
    P=dot_P*te+P;

    //std::cout<<"debug dot_P\n"<<dot_P<<std::endl;
    //std::cout<<"debug P \n"<<P<<std::endl;


    VectorXd dot_hat_z(5);

    Vector4d y_hat;
    y_hat<<hat_z(0),hat_z(1),hat_z(2),hat_z(3);

    dot_hat_z=g+K*(y_v-y_hat);
    hat_z=dot_hat_z*te+hat_z;

    //std::cout<<"y "<<y[0]<<" "<<y[1]<<" "<<y[2]<<" "<<y[3]<<" "<<std::endl;
    //std::cout<<"y  "<<y<<std::endl;
    //std::cout<<"hat_z  \n"<<hat_z<<std::endl;

    return hat_z;

}
