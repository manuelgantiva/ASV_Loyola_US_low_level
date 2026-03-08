#include "asv_library/curvas_sim.h"
#include <cmath> 

void recta(double w, Target* px, double ax, double by, double mx, double my){
    px->yp = by + my * w;
    px->xp = ax + mx * w;
    px->dyp = my;
    px->dxp = mx;
    double dyp2 = 0;
    double dxp2 = 0;
    px->phip = atan2(px->dyp,px->dxp);
    px->dphip = (dyp2*px->dxp - dxp2*px->dyp)/(px->dyp*px->dyp + px->dxp*px->dxp);
    px->f_c = std::sqrt(px->dyp*px->dyp + px->dxp*px->dxp);
}

void arco(double w, Target* px, double ax, double by, double r, bool sig) {
    double dyp2 = 0;
    if(sig){
        px->yp = by + r * cos(w);
        px->dyp = -r * sin(w);
        dyp2 = -r * cos(w);
    }else{
        px->yp = by - r * cos(w);
        px->dyp = r * sin(w);
        dyp2 = r * cos(w);
    }
    px->xp = ax + r * sin(w);
    px->dxp = r * cos(w);
    double dxp2 = -r * sin(w);
    px->phip = atan2(px->dyp,px->dxp);
    px->dphip = (dyp2*px->dxp - dxp2*px->dyp)/(px->dyp*px->dyp + px->dxp*px->dxp);
    px->f_c = std::sqrt(px->dyp*px->dyp + px->dxp*px->dxp);
}


Target circle_30m(double w)
{
    Target tar_p;
    tar_p.xp  = 37.0 - 30.0 * std::cos(w);
    tar_p.yp  =  7.0 + 30.0 * std::sin(w);
    tar_p.dxp = 30.0 * std::sin(w);
    tar_p.dyp = 30.0 * std::cos(w);
    const double dxp2 = 30.0 * std::cos(w);
    const double dyp2 = -30.0 * std::sin(w);
    tar_p.phip = std::atan2(tar_p.dyp, tar_p.dxp);
    const double denom = tar_p.dxp * tar_p.dxp + tar_p.dyp * tar_p.dyp;
    tar_p.dphip = (denom > 0.0) ? (tar_p.dxp * dyp2 - tar_p.dyp * dxp2) / denom : 0.0;
    tar_p.f_c = std::sqrt(denom);
    return tar_p;
}

Target line_northeast(double w)
{
    Target tar_p;
    tar_p.xp  = w + 7.0;
    tar_p.yp  = w + 7.0;
    tar_p.dxp = 1.0;
    tar_p.dyp = 1.0;
    const double dxp2 = 0.0;
    const double dyp2 = 0.0;
    tar_p.phip = std::atan2(tar_p.dyp, tar_p.dxp);
    const double denom = tar_p.dxp * tar_p.dxp + tar_p.dyp * tar_p.dyp;
    tar_p.dphip = (denom > 0.0) ? (tar_p.dxp * dyp2 - tar_p.dyp * dxp2) / denom : 0.0;
    tar_p.f_c = std::sqrt(denom);
    return tar_p;
}
