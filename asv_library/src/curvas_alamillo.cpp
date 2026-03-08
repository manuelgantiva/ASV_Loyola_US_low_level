#include "asv_library/curvas_alamillo.h"
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

//Trayectoria interna
Target curva_ala_1_2(double w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0, 1, -2.0,  0);
    }else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w - 12.5 + 3.12159265, &result,  -31.0, 3.0, 2.0, true);
    }else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0 , 5.0 , 2.0, 0);
    }else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w - 28.14159265 , &result,  -6.0, 3.0, 2.0, true);
    }
    
    return result;
}

//Trayectoria central
Target curva_ala_1_4(double w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0, -1.0, -2, 0);
    } else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w - 12.5 + 3.12159265, &result,  -31.0, 3.0, 4.0, true);
    }else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0 , 7, 2, 0);
    }else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w - 28.14159265 , &result,  -6.0, 3.0, 4.0, true);
    }
    return result;
}

//Trayectoria externa
Target curva_ala_1_6(double w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0,-3.0, - 2.0, 0);
    }else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w - 12.5 + 3.12159265, &result,  -31.0, 3.0, 6.0, true);
    }else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0 , 9.0 ,  2.0, 0);
    }else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w - 28.14159265 , &result,  -6.0, 3.0, 6.0, true);
    }
    return result;
}


Target line_south(double w)
{
    Target tar_p;
    tar_p.xp  = -w;
    tar_p.yp  = -1.0;
    tar_p.dxp = -1.0;
    tar_p.dyp = 0.0;
    const double dxp2 = 0.0;
    const double dyp2 = 0.0;
    tar_p.phip = std::atan2(tar_p.dyp, tar_p.dxp);
    const double denom = tar_p.dxp * tar_p.dxp + tar_p.dyp * tar_p.dyp;
    tar_p.dphip = (denom > 0.0) ? (tar_p.dxp * dyp2 - tar_p.dyp * dxp2) / denom : 0.0;
    tar_p.f_c = std::sqrt(denom);
    return tar_p;
}

Target circle_10m(double w)
{
    Target tar_p;
    tar_p.xp  = -11.0 + 10.0 * std::cos(w);
    tar_p.yp  =   2.0 - 10.0 * std::sin(w);
    tar_p.dxp = -10.0 * std::sin(w);
    tar_p.dyp = -10.0 * std::cos(w);
    const double dxp2 = -10.0 * std::cos(w);
    const double dyp2 =  10.0 * std::sin(w);
    tar_p.phip = std::atan2(tar_p.dyp, tar_p.dxp);
    const double denom = tar_p.dxp * tar_p.dxp + tar_p.dyp * tar_p.dyp;
    tar_p.dphip = (denom > 0.0) ? (tar_p.dxp * dyp2 - tar_p.dyp * dxp2) / denom : 0.0;
    tar_p.f_c = std::sqrt(denom);
    return tar_p;
}

Target circle_8m(double w)
{
    Target tar_p;
    tar_p.xp  = -9.0 + 8.0 * std::cos(w);
    tar_p.yp  =  2.0 - 8.0 * std::sin(w);
    tar_p.dxp = -8.0 * std::sin(w);
    tar_p.dyp = -8.0 * std::cos(w);
    const double dxp2 = -8.0 * std::cos(w);
    const double dyp2 =  8.0 * std::sin(w);
    tar_p.phip = std::atan2(tar_p.dyp, tar_p.dxp);
    const double denom = tar_p.dxp * tar_p.dxp + tar_p.dyp * tar_p.dyp;
    tar_p.dphip = (denom > 0.0) ? (tar_p.dxp * dyp2 - tar_p.dyp * dxp2) / denom : 0.0;
    tar_p.f_c = std::sqrt(denom);
    return tar_p;
}

Target lissajous_10m(double w)
{
    Target tar_p;
    const double p2 = 1.5707963268;
    tar_p.xp  = -10.0 + 10.0 * std::sin(2.0 * w);
    tar_p.yp  =  -4.0 + 15.0 * std::sin(w + p2);
    tar_p.dxp =  20.0 * std::cos(2.0 * w);
    tar_p.dyp =  15.0 * std::cos(w + p2);
    const double dxp2 = -40.0 * std::sin(2.0 * w);
    const double dyp2 = -15.0 * std::sin(w + p2);
    tar_p.phip = std::atan2(tar_p.dyp, tar_p.dxp);
    const double denom = tar_p.dxp * tar_p.dxp + tar_p.dyp * tar_p.dyp;
    tar_p.dphip = (denom > 0.0) ? (tar_p.dxp * dyp2 - tar_p.dyp * dxp2) / denom : 0.0;
    tar_p.f_c = std::sqrt(denom);
    return tar_p;
}

Target lissajous_5m(double w)
{
    Target tar_p;
    const double p2 = 1.5707963268;
    tar_p.xp  = -5.0 + 5.0 * std::sin(2.0 * w);
    tar_p.yp  = -4.0 + 15.0 * std::sin(w + p2);
    tar_p.dxp = 10.0 * std::cos(2.0 * w);
    tar_p.dyp = 15.0 * std::cos(w + p2);
    const double dxp2 = -20.0 * std::sin(2.0 * w);
    const double dyp2 = -15.0 * std::sin(w + p2);
    tar_p.phip = std::atan2(tar_p.dyp, tar_p.dxp);
    const double denom = tar_p.dxp * tar_p.dxp + tar_p.dyp * tar_p.dyp;
    tar_p.dphip = (denom > 0.0) ? (tar_p.dxp * dyp2 - tar_p.dyp * dxp2) / denom : 0.0;
    tar_p.f_c = std::sqrt(denom);
    return tar_p;
}


