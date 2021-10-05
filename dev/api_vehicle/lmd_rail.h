#ifndef LMD_RAIL_H_INCLUDED
#define LMD_RAIL_H_INCLUDED


#include <string>
#include "lmd_cubic_spline.h"
#include "../pugi/src/pugixml.hpp"

class Marker;
class CubicSpline2D;
class CubicSpline3D;
struct EntityRail;
class TrackArrays;

class Rail {
 public:
   Rail(pugi::xml_node ni, CubicSpline2D *rail_profile_right_,
        const TrackArrays *tr_);
   void CreateCurrSplines(const double time, const double tend);
   void CreateVelSplines();
   CubicSpline3D* CreateOneSpline(
      const double lamda,
      const double *xan , const double *yan , const double *zan,
      const double *xan1, const double *yan1, const double *zan1);



   Vector PointOnRail(const double xt, const double xr,
                      const bool is_right, const unsigned nstate) const;
   Matrix RailCrossSectionRotmat(const double xt, const bool is_right,
                                 const unsigned nstate) const;
   Matrix RailCrossSectionRotmat(const CubicSpline3D*, const double xt,
                                 const bool is_right);
   Matrix RailCrossSectionRotmatDer(const double xt, const bool is_right,
                                    const unsigned nstate) const;
   CubicSpline3D* GetTrackSpline(const bool is_right) const;
   inline CubicSpline2D* GetProfileSpline() const {
     return rail_profile_right;
   }

 //private:
  int id;
  std::string label;
  CubicSpline2D *rail_profile_right;
  CubicSpline3D *track_pos_left;
  CubicSpline3D *track_pos_right;
  CubicSpline3D *track_vel_left;
  CubicSpline3D *track_vel_right;
  const TrackArrays *tr;
  int front_veh_id;
  int number_of_wagons;
  double xa;
  double ya;
  double za;
  double xa_b1;
  double ya_b1;
  double za_b1;
  double xa_b2;
  double ya_b2;
  double za_b2;
  std::vector<int> vehicles;
  std::vector<int> bogies;
  CubicSpline2D *vx_track_left;
  CubicSpline2D *vy_track_left;
  CubicSpline2D *vz_track_left;
  CubicSpline2D *vx_track_right;
  CubicSpline2D *vy_track_right;
  CubicSpline2D *vz_track_right;

};



#endif // LMD_RAIL_H_INCLUDED
