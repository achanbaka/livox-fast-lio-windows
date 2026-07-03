#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "types.h"
#include "livox_sdk.h"

using namespace std;

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

enum LID_TYPE{AVIA = 1, VELO16, OUST64, MARSIM}; //{1, 2, 3}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

struct orgtype
{
  double range;
  double dista; 
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

class Preprocess
{
  public:
  Preprocess();
  ~Preprocess();
  
  // Process Livox raw data (replaces ROS CustomMsg version)
  // data: array of LivoxEthPacket, num: count, timestamp: seconds
  void process(const LivoxEthPacket* data, uint32_t num, double timestamp, PointCloudXYZI::Ptr &pcl_out);
  
  // Process pre-parsed point arrays (used by lvx_reader)
  void process_points(const float* x, const float* y, const float* z,
                      const uint8_t* line, const uint8_t* tag, const uint32_t* offset_time,
                      const uint8_t* reflectivity,
                      uint32_t num, double timestamp, PointCloudXYZI::Ptr &pcl_out);

  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  bool feature_enabled, given_offset_time;
    
  private:
  void avia_handler(const float* x, const float* y, const float* z,
                    const uint8_t* line, const uint8_t* tag, const uint32_t* offset_time,
                    const uint8_t* reflectivity,
                    uint32_t num, double timestamp);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, unsigned int i, unsigned int &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, unsigned int i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
#endif
