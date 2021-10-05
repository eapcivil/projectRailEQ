#ifndef SOLUTION_PARAMS_H_INCLUDED
#define SOLUTION_PARAMS_H_INCLUDED

#include <string>
#include <vector>
class PostProcessing;


#pragma pack(push, 1)
struct SP {
  //SP() : post_processing(nullptr) { }
  // ---------------------------------------------------------input_from_python
  static bool is_first_time;
  static double tend;
  static double nintervals;
  static double h_init;
  static double tol_in;
  static double tol_out;
  static double tol_dphi;
  static double tol_phi;
  static double nn;
  static double krr_explicit;
  static double crr_explicit;
  static const char *xml1_path;
  static const char *xml2_path;
  static int npoints;
  static double vx;

  // ---------------------------------------------------------------------yours
  static int imax;
  static int jmax;
  static bool do_write_to_plt;
  static bool do_write_to_vtk;
  static double print_interval;
  static std::string plt_name;
  static PostProcessing* post_processing;
  static double dist_rail;
  static double scale_pos;
  static double scale_vel;
  static double track_offset_z;
  static double track_start_behind;
  static double wheel_radius;

  //bool is_varstep;
  //int j_dec = jmax;
  //int j_inc = 3;
  //double h_redu;
  //double h_min;
  //double h_max = 1e-2;

  //PostProcessing *post_processing;
  // --------------------------------------------------------------------OTHERS
  static double KTIME_FOR_PRINT;
  static double KSMALL_NUMBER;
  static double KSMALL_NUMBER_FOR_EXPM;
  static double KTOL_FOR_TEND;
  static double KTOL_FOR_REORTHOGONALIZATION;  // for RotmatToEuler
  static bool   KEULER_ANGLES_IN_DEG;

  // -----------------------------------------------------------------------VTK
  static int KNUMBER_OF_SEGMENTS_FOR_VTK_FRUSTUM;
  static int KNUMBER_OF_SEGMENTS_FOR_VTK_RAIL_POLYGON;
  static int KNUMBER_OF_SEGMENTS_FOR_VTK_RAIL_EXTRUSION;
  static int KNUMBER_OF_SEGMENTS_FOR_VTK_WHEEL_POLYLINE;
  static int KNUMBER_OF_SEGMENTS_FOR_VTK_WHEEL_REVOLUTION;

};
#pragma pack(pop)

#endif // SOLUTION_PARAMS_H_INCLUDED
