#ifndef VEHICLE_H_INCLUDED
#define VEHICLE_H_INCLUDED
#ifdef VEHICLE_EXPORTS
#define VEHICLE_API __declspec(dllexport)
#else
#define VEHICLE_API __declspec(dllimport)
#endif

extern "C" VEHICLE_API void Vehicle(
  const bool cosim_converge, const bool is_first_time,
  const double t0, const double t1, const double nintervals,
  const double tol_in, const double tol_out,
  const double tol_dphi, const double tol_phi,
  const double nn, const double krr_explicit, const double crr_explicit,
  const char* xml0_path, const char* xml1_path, const char* xml2_path,
  const int npoints, const double vx,
  const double *x_p_l_n, const double *y_p_l_n, const double *z_p_l_n,
  const double *x_p_l_n1, const double *y_p_l_n1, const double *z_p_l_n1,
  const double *x_p_r_n, const double *y_p_r_n, const double *z_p_r_n,
  const double *x_p_r_n1, const double *y_p_r_n1, const double *z_p_r_n1,
  const double *x_v_l_n, const double *y_v_l_n, const double *z_v_l_n,
  const double *x_v_l_n1, const double *y_v_l_n1, const double *z_v_l_n1,
  const double *x_v_r_n, const double *y_v_r_n, const double *z_v_r_n,
  const double *x_v_r_n1, const double *y_v_r_n1, const double *z_v_r_n1,
  double *cp_positions, double *cp_velocities, double *cp_forces,
  double *vehicle_accelerations, double *bogie_accelerations,
  double *bogie_yaws);

// cosim_converge : cosimulation converge
// is_first_time : is the first you are calling vehicle
// t0 : previous cosimulation time
// t1 : current  cosimulation time
// nintervals : number of intervals for vehicle solution, thus time-step = (t1 - t0)/nintervals
// tol_in : velocity tolerance (1e-3 or lower)
// tol_out : residual tolerance (1e-2 or lower), must be tol_in < tol_out
// tol_dphi : tolerance for dphi_dt (1e-3 or lower)
// tol_phi : tolerance for phi (1e-2 or lower)

// nn:
//  if nn > 0.0 then krrs = ( 2.0*PI/(nn * h) )^2 * mrrs, crrs = 2.0 * sqrt(krrs * mrrs);
//  if nn == 0.0 or something very small (e.g. 1e-10) then krrs = crrs = 0
//  if nn < 0.0 then krrs = krr_explicit, crrs = crr_explicit

// xml1_path : path for xml1, THIS IS ALWAYS THE INPUT FOR THE DLL TO SOLVE
// xml0_path : path for xml0, used as input (copying it to xml1) only when is_first_time = true
// xml1_path : path for xml1, used as input when is_first_time = false, this means that at least
//             one cosim_converge has occured
// xml2_path : path for xml2, used to keep the results from the xml1_run in case there is cosim_converge

// npoints : number of interpolation points for the position-velocity splines
// vx : the constant forward velocity of the front vehicle

// x_p_l_n  : x coords of interpolation points, for position spline, left rail, spline at n
// y_p_l_n  : y coords of interpolation points, for position spline, left rail, spline at n
// z_p_l_n  : z coords of interpolation points, for position spline, left rail, spline at n
// x_p_l_n1 : x coords of interpolation points, for position spline, left rail, spline at n + 1
// y_p_l_n1 : y coords of interpolation points, for position spline, left rail, spline at n + 1
// z_p_l_n1 : z coords of interpolation points, for position spline, left rail, spline at n + 1

// x_p_r_n  : x coords of interpolation points, for position spline, right rail, spline at n
// y_p_r_n  : y coords of interpolation points, for position spline, right rail, spline at n
// z_p_r_n  : z coords of interpolation points, for position spline, right rail, spline at n
// x_p_r_n1 : x coords of interpolation points, for position spline, right rail, spline at n + 1
// y_p_r_n1 : y coords of interpolation points, for position spline, right rail, spline at n + 1
// z_p_r_n1 : z coords of interpolation points, for position spline, right rail, spline at n + 1

// x_v_l_n  : x coords of interpolation points, for velocity spline, left rail, spline at n
// y_v_l_n  : y coords of interpolation points, for velocity spline, left rail, spline at n
// z_v_l_n  : z coords of interpolation points, for velocity spline, left rail, spline at n
// x_v_l_n1 : x coords of interpolation points, for velocity spline, left rail, spline at n + 1
// y_v_l_n1 : y coords of interpolation points, for velocity spline, left rail, spline at n + 1
// z_v_l_n1 : z coords of interpolation points, for velocity spline, left rail, spline at n + 1

// x_v_r_n  : x coords of interpolation points, for velocity spline, right rail, spline at n
// y_v_r_n  : y coords of interpolation points, for velocity spline, right rail, spline at n
// z_v_r_n  : z coords of interpolation points, for velocity spline, right rail, spline at n
// x_v_r_n1 : x coords of interpolation points, for velocity spline, right rail, spline at n + 1
// y_v_r_n1 : y coords of interpolation points, for velocity spline, right rail, spline at n + 1
// z_v_r_n1 : z coords of interpolation points, for velocity spline, right rail, spline at n + 1

// cp_positions : xyz coords of the contact points (cp one per wheel, thus two per wheelset)
    // example for two wheelsets: cp_positions = { x_first_wheelset_left_wheel,
    //                                             y_first_wheelset_left_wheel,
    //                                             z_first_wheelset_left_wheel,
    //                                             x_first_wheelset_right_wheel,
    //                                             y_first_wheelset_right_wheel,
    //                                             z_first_wheelset_right_wheel,
    //                                             x_second_wheelset_left_wheel,
    //                                             y_second_wheelset_left_wheel,
    //                                             z_second_wheelset_left_wheel,
    //                                             x_second_wheelset_right_wheel,
    //                                             y_second_wheelset_right_wheel,
    //                                             z_second_wheelset_right_wheel }
// cp_velocities : xyz velocity components of the contact points
    // example for two wheelsets: cp_velocities = "see cp_positions"
// cp_forces : xyz force components at the contact points
    // example for two wheelsets: cp_forces = "see cp_positions"
// vehicle_accelerations: see "explanation_new outputs.pdf"
// bogie_accelerations: see "explanation_new outputs.pdf"
// bogie_yaws: see "explanation_new outputs.pdf"

#endif // VEHICLE_H_INCLUDED

