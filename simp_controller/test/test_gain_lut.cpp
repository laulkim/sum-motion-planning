#include "simp_controller/control.hpp"
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <cstdio>
#include <unistd.h>

using namespace simp_controller;
void require(bool ok) { if (!ok) throw std::runtime_error("LUT test failed"); }
void near(double a, double b) { require(std::isfinite(a) && std::abs(a-b) < 1e-10); }
template<class F> void rejects(F f) {
  bool rejected = false;
  try { f(); } catch (const std::exception&) { rejected = true; }
  require(rejected);
}
Gains analytic(double x, double y, double z) {
  return {5 + x + 2*y + z, 8 + 2*x + y + 3*z, 4 + x + y + z};
}
void check(const Gains& g, const Gains& expected) {
  near(g.kx, expected.kx); near(g.ky, expected.ky); near(g.ktheta, expected.ktheta);
}
int main() {
 try {
  const std::array<std::vector<double>,3> axes{{{-1,0,3}, {0,2}, {-0.5,0,1.5}}};
  std::vector<Gains> gains;
  for (double x : axes[0]) for (double y : axes[1]) for (double z : axes[2])
    gains.push_back(analytic(x,y,z));
  const GainLut lut(axes, gains);
  for (double x : axes[0]) for (double y : axes[1]) for (double z : axes[2])
    check(lut.lookup(x,y,z), analytic(x,y,z));
  check(lut.lookup(1.5,0,0), analytic(1.5,0,0));  // x only
  check(lut.lookup(0,1,0), analytic(0,1,0));       // y only
  check(lut.lookup(0,0,0.75), analytic(0,0,0.75)); // yaw only
  check(lut.lookup(-0.25,1.25,0.2), analytic(-0.25,1.25,0.2));
  check(lut.lookup(-100,-100,-100), analytic(-1,0,-0.5));
  check(lut.lookup(100,100,100), analytic(3,2,1.5));
  Point p;
  p.yaw_rate = 0.2;
  auto scheduled = scheduled_gains(p, Gains{1,2,3}, &lut);
  near(scheduled.kx,0); near(scheduled.ky,0); near(scheduled.ktheta,4.2);
  p.yaw_rate = 0;
  check(scheduled_gains(p, Gains{}, &lut), analytic(0,0,0)); // full stop
  const GainLut constant(axes, std::vector<Gains>(18,Gains{1,2,3}));
  p.vx=0.7; p.vy=-0.4; p.yaw_rate=0.1;
  check(scheduled_gains(p,Gains{},&constant), Gains{1,2,3});
  const Error error{0.2,-0.3,0.4};
  const auto fixed = lyapunov_control(p,error,Gains{1,2,3});
  const auto lookup = lyapunov_control(p,error,scheduled_gains(p,Gains{},&constant));
  near(fixed.linear.x,lookup.linear.x); near(fixed.linear.y,lookup.linear.y);
  near(fixed.angular.z,lookup.angular.z);
  p.vx=p.vy=0;
  const auto rotation=lyapunov_control(p,error,scheduled_gains(p,Gains{},&lut));
  near(rotation.linear.x,0); near(rotation.linear.y,0);
  near(rotation.angular.z,0.1+4.1*0.4);

  // Integration through reference selection + prediction + control_cycle.
  auto trajectory=std::make_shared<Trajectory>();
  trajectory->header.frame_id="odom"; trajectory->header.stamp.sec=1;
  trajectory->points={p}; trajectory->points[0].x=2; trajectory->points[0].y=3;
  trajectory->points[0].body_yaw=0.2;
  auto odom=std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.frame_id="odom"; odom->header.stamp.sec=1;
  odom->child_frame_id="base_link"; odom->pose.pose.orientation.w=1;
  const auto cycle=control_cycle({trajectory,odom},1000000000,10000000,Gains{1,2,3},"base_link",&lut);
  require(cycle.tracking.valid);
  near(cycle.velocity.linear.x,0); near(cycle.velocity.linear.y,0);
  near(cycle.velocity.angular.z,0.1+4.1*0.2);

  const double nan=std::numeric_limits<double>::quiet_NaN();
  rejects([&]{lut.lookup(nan,0,0);});
  rejects([&]{GainLut(axes,{});});
  auto wrong=axes; wrong[0]={0,0}; rejects([&]{GainLut(wrong,gains);});
  wrong=axes; wrong[0]={3,0,-1}; rejects([&]{GainLut(wrong,gains);});
  wrong=axes; wrong[1][0]=nan; rejects([&]{GainLut(wrong,gains);});
  auto invalid=gains; invalid[0].kx=-1; rejects([&]{GainLut(axes,invalid);});
  invalid=gains; invalid[0].ktheta=nan; rejects([&]{GainLut(axes,invalid);});
  const GainLut singleton({{{0},{0},{0}}},{Gains{1,2,3}});
  check(singleton.lookup(-1,2,3),Gains{1,2,3});

  const auto path=std::string("/tmp/simp_lut_test_")+std::to_string(getpid())+".csv";
  {
    std::ofstream file(path);
    file << "vx,vy,yaw_rate,kx,ky,ktheta\n";
    for (double x : axes[0]) for (double y : axes[1]) for (double z : axes[2]) {
      const auto g=analytic(x,y,z);
      file << x << ',' << y << ',' << z << ',' << g.kx << ',' << g.ky << ',' << g.ktheta << '\n';
    }
  }
  check(GainLut::load_csv(path).lookup(0.2,0.3,0.4),analytic(0.2,0.3,0.4));
  for (const auto* bad : {
    "wrong\n", "vx,vy,yaw_rate,kx,ky,ktheta\n0,0,0,1,2,nan\n",
    "vx,vy,yaw_rate,kx,ky,ktheta\n0,0,0,1,2,3\n0,0,0,1,2,3\n",
    "vx,vy,yaw_rate,kx,ky,ktheta\n1,0,0,1,2,3\n0,0,0,1,2,3\n",
    "vx,vy,yaw_rate,kx,ky,ktheta\n0,0,0,1,2,3,\n",
    "vx,vy,yaw_rate,kx,ky,ktheta\n0,0,0,1,2,3junk\n",
    "vx,vy,yaw_rate,kx,ky,ktheta\n0,0,0,1,2,3\n1,1,1,1,2,3\n"}) {
    {std::ofstream file(path); file << bad;}
    rejects([&]{GainLut::load_csv(path);});
  }
  std::remove(path.c_str());
  std::cout << "3D gain LUT: interpolation, nonuniform axes, clamps, file validation, fixed parity and spot constraint passed\n";
 } catch(const std::exception& error) {std::cerr << error.what() << '\n';return 1;}
}
