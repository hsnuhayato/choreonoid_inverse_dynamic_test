#include <stdio.h>
#include <iostream>
#include <fstream>
#include <deque>

#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/VRMLBodyLoader>
#include <cnoid/EigenTypes>
#include <cnoid/EigenUtil>
#include <cnoid/Link>
#include <cnoid/src/Body/InverseDynamics.h>

using namespace std;
using namespace cnoid;
//using namespace Eigen

// log is in the order of [tau(by choreonoid), tau(by lagrange), q]
std::ofstream ofs("ref_tau.log");

#define NEAR0 1e-8

template<class T>
void Interplation5(std::deque<T>& input, const T ps, const T dps, const T ddps,
                   const T pf, const T dpf, const T ddpf,
                   const double tf,
                   const double dt=0.005, const double start_time = 0, double end_time = 0)
{
  T a0,a1,a2,a3,a4,a5;
  if (end_time == 0) {
    end_time = tf;
  }
  int start_idx = (int)(start_time/dt+NEAR0) + 1;
  int end_idx = (int)(end_time/dt+NEAR0) + 1;

  a0=ps;
  a1=dps;
  a2=ddps/2.0;
  a3=(20.0*pf - 20.0*ps - (8.0*dpf + 12.0*dps)*tf - (3.0*ddps - ddpf)*pow(tf,2))/(2.0*pow(tf,3));
  a4=(30.0*ps - 30.0*pf + (14.0*dpf + 16.0*dps)*tf + (3.0*ddps - 2.0*ddpf)*pow(tf,2))/(2.0*pow(tf,4));
  a5=(12.0*pf - 12.0*ps - (6.0*dpf + 6.0*dps)*tf - (ddps - ddpf)*pow(tf,2))/(2.0*pow(tf,5));

  for(int i=start_idx; i<end_idx; i++) {
    double ti=dt*i;
    T tmp;
    tmp=a0+a1*ti+a2*pow(ti,2)+a3*pow(ti,3)+a4*pow(ti,4)+a5*pow(ti,5);
    input.push_back(tmp);
  }
}


int main(int argc, char *argv[]) {
  double tar_q = 90.0;
  double tar_time = 2.0;
  double dt = 0.005;
  BodyPtr robot;
  BodyLoader bl;
  string model_url = "../1linkbot.yaml";
  robot = bl.load(model_url.c_str());

  if (!robot) {
    model_url = "1linkbot.yaml";
    robot = bl.load(model_url.c_str());
    if (!robot) {
      cerr << "load fail" << endl;
      return -1;
    }
  }
  
  robot -> joint(0) -> q() = 0;
  robot -> calcForwardKinematics();
  
  cnoid::Vector3d::Zero();
  double tau_lagrange = 0.0;
  double q_pre = 0.0;
  double v_pre = 0.0;
  cnoid::Vector3d gv(0.0, 0.0, -9.80665);
  std::deque<double> q_deque;
  Interplation5(q_deque, 0.0, 0.0, 0.0, cnoid::radian(tar_q), 0.0, 0.0, tar_time, dt);

  while (!q_deque.empty()) {
    robot -> joint(0) -> q() = q_deque.at(0);
    robot -> joint(0) -> dq()  = (robot -> joint(0) -> q() - q_pre) / dt;
    robot -> joint(0) -> ddq() = (robot -> joint(0) -> dq() - v_pre) / dt;
    q_pre = robot -> joint(0) -> q();
    v_pre = robot -> joint(0) -> dq();
    q_deque.pop_front();
    
    robot->calcForwardKinematics(true, true);
    robot -> joint(0) -> tau_ext() = cnoid::Vector3d::Zero();
    robot -> joint(0) -> f_ext() = cnoid::Vector3d::Zero();
    robot -> joint(0) -> addExternalForce(gv * robot->joint(0)->m(), robot->joint(0)->c());
    
    calcInverseDynamics(robot->rootLink());

    // lagrange calculate. tau = ddq * (I + c^2 + m) + c * g * m * sin(q)
    tau_lagrange =  robot -> joint(0) -> ddq() *
      (robot->joint(0)->I()(1,1) + robot->joint(0)->c()[2] * robot->joint(0)->c()[2] * robot->joint(0)->m()) +
      fabs(robot->joint(0)->c()[2]) * 9.80665 * robot->joint(0)->m() * sin(robot->joint(0)->q());

    ofs << robot -> joint(0) -> u() << " " <<   tau_lagrange <<
      " " <<  robot -> joint(0) -> q() << endl;

    assert(fabs(robot -> joint(0) -> u() - tau_lagrange) < NEAR0);
  }

  cout << "test ok" << endl;
  
  return 0;
}
