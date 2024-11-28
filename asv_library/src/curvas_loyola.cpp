#include "asv_library/curvas_loyola.h"
#include <cmath> 

void recta(float w, Target* px, float ax, float by, float mx, float my){
    px->yp = by + my * w;
    px->xp = ax + mx * w;
    px->dyp = my;
    px->dxp = mx;
}

void arco(float w, Target* px, float ax, float by, float r, bool sig) {
    if(sig){
        px->yp = by + r * cos(w);
        px->dyp = -r * sin(w);
    }else{
        px->yp = by - r * cos(w);
        px->dyp = r * sin(w);
    }
    px->xp = ax + r * sin(w);
    px->dxp = r * cos(w);
}

Target curva_2_i(const float w) {
    Target result;
    if (w < 10.077822185373194) {
        // Trayectoria 1: Línea recta 
        recta(w, &result, 0.5, 4.5, -1.8357140719202854, -0.7938223013709336);
        //recta(w, &result, 0, 0, -1, 0);
    } 
    else if (w >= 10.077822185373194 && w < 11.648618512 ) {
        // Trayectoria 2: Cuarto de circunferencia 
        arco(w - 10.077822185373194 + 2.7334435498347043, &result, -18.793822301370934, -1.6642859280797144,  2.0, true);
    }
    else if (w >= 11.648618512) {
        // Trayectoria 3: Línea recta 
        recta(w - 11.648618512, &result, -20.62953637329122, -2.458108229450648, -0.793822301370952, 1.8357140719202774);
    }
    
    return result;
}

Target curva_2_c(const float w) {
    Target result;
    if (w < 5.038911092686587) {
        // Trayectoria 1: Línea recta 
        recta(w, &result, 1.29, 2.66 , -3.6714281438405694, -1.5876446027418702);
    } 
    else if (w >= 5.038911092686587 && w < 6.609707417) {
        // Trayectoria 2: Cuarto de circunferencia 
        arco(w - 5.038911092686587 + 2.7334435498347034, &result, -18.79764460274187, -1.6685718561594307, 4.0, true);
    }
    else if (w >= 6.609707417) {
        // Trayectoria 3: Línea recta 
        recta(w - 6.609707417, &result, -22.469072746582437 , -3.256216458901301, -1.587644602741868, 3.6714281438405703);
    }
    
    
    return result;
}


Target curva_2_d(const float w) {
    Target result;
    if (w < 3.3592740617910617) {
        // Trayectoria 1: Línea recta 
        recta(w, &result, 2.08, 0.8200000000000001, -5.507142215760855, -2.381466904112803);
    } 
    else if (w >=3.3592740617910617 && w < 4.930070389) {
        // Trayectoria 2: Cuarto de circunferencia 
        arco(w - 3.3592740617910617 + 2.733443549834704, &result, -18.801466904112804, -1.6728577842391446, 6.0, true);
    }
    else if (w >= 4.930070389) {
        // Trayectoria 3: Línea recta 
        recta(w - 4.930070389, &result, -24.30860911987366, -4.05432468835195, -2.3814669041128056, 5.5071422157608545);
    }
    
    
    return result;
}