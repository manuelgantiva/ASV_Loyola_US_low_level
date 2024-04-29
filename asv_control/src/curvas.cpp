#include "curvas.hpp"
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

// Semicircle of radius 6
Target curva1(float w) {
    Target result;
    if (w < 5.60258) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, 0.0, -5.53317, -2.32036);
    } else if (w >= 5.60258 && w < 8.74418) {
        // Trajectory 2: 180 degree arc of circumference
        arco(w - 5.60258 + 2.74926, &result, -33.32036, -7.46683 , 6.0, true);
    } else if (w >= 8.74418) {
        // Trajectory 3: 15 meter straight line
        recta(w - 8.74418, &result, -35.64072, -1.93367, 5.53317, 2.32036);
    }   
    return result;
}

// Semicircle of radius 5
Target curva2(float w) {
    Target result;
    if (w < 6.72309) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, 0.0, -4.61097, -1.93363);
    } else if (w >= 6.72309 && w < 9.86469) {
        // Trajectory 2: 180 degree arc of circumference
        arco(w - 6.72309 + 2.74925, &result, -32.93363, -8.38903 , 5.0, true);
    } else if (w >= 9.86469) {
        // Trajectory 3: 15 meter straight line
        recta(w - 9.86469, &result, -34.86727, -3.77806, 4.61097, 1.93363);
    } 
    return result;
}

// Semicircle of radius 4
Target curva3(float w) {
    Target result;
    if (w < 8.40387) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, 0.0, -3.68878, -1.54691);
    } else if (w >= 8.40387 && w < 11.54546) {
        // Trajectory 2: 180 degree arc of circumference
        arco(w - 8.40387 + 2.74925, &result, -32.54691, -9.31122 , 4.0, true);
    } else if (w >= 11.54546) {
        // Trajectory 3: 15 meter straight line
        recta(w - 11.54546, &result, -34.09381, -5.62244, 3.68878, 1.54691);
    } 
    return result;
}

// Semicircle of radius 3
Target curva4(float w) {
    Target result;
    while(w >= 28.70134){
        w = w - 28.70134;
    }
    if (w < 11.20516) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, 0.0, -2.76658, -1.16018);
    } else if (w >= 11.20516 && w < 14.34675) {
        // Trajectory 2: 90 degree arc of circumference
        arco(w - 11.20516 + 2.74925, &result, -32.16018, -10.23342 , 3.0, true);
    } else if (w >= 14.34675 && w < 25.55675) {
        // Trajectory 3: 11.21 meter straight line
        recta(w - 14.34675, &result, -33.32036, -7.46683, 2.76658, 1.16018);
    } else if (w >= 25.55675 && w < 28.70134) {
        // Trajectory 4: 90 degree arc of circumference
        arco(w - 25.55675 + 5.89085, &result, -1.14678, 2.77220 , 3.0, true);
    } 
    return result;
}

// Semicircle of radius 2
Target curva5(float w) {
    Target result;
    while(w >= 30.05804){
        w = w - 30.05804;
    }
    if (w < 11.88486) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, 0.0, -1.85109, -0.75727);
    } else if (w >= 11.88486 && w < 15.02645) {
        // Trajectory 2: 90 degree arc of circumference
        arco(w - 11.88486 + 2.74925, &result, -22.75727, -7.14891 , 2.0, true);
    } else if (w >= 15.02645 && w < 26.91645) {
        // Trajectory 3: 11.89 meter straight line
        recta(w - 15.02645, &result, -23.51453, -5.29781, 1.85109, 0.75727);
    } else if (w >= 26.91645 && w < 30.05804) {
        // Trajectory 4: 90 degree arc of circumference
        arco(w - 26.91645 + 5.89085, &result, -0.74776, 1.85498 , 2.0, true);
    } 
    return result;
}

// Quarter circle of radius 2
Target curva6(float w) {
    Target result;
    if (w < 11.88486) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, 0.0, -1.85109, -0.75727);
    } else if (w >= 11.88486 && w < 13.45968) {
        // Trajectory 2: 90 degree arc of circumference
        arco(w - 11.88486 + 2.74925, &result, -22.75727, -7.14891 , 2.0, true);
    } else if (w >= 13.45968 && w < 21.45968) {
        // Trajectory 3: 16 meter straight line
        recta(w - 13.45968, &result, -24.60836, -7.90617, -0.75727, 1.85109);
    } else if (w >= 21.45968 && w < 23.03048) {
        // Trajectory 4: 90 degree arc of circumference
        arco(w - 21.45968 - 1.95912, &result, -28.81539, 7.65985 , 2.0, true);
    } else if (w >= 23.03048) {
        // Trajectory 5: 16 meter straight line
        recta(w - 23.03048, &result, -29.57266, 9.51094, 1.85109, 0.75727);
    } 
    
    return result;
}

// Quarter circle of radius 3
Target curva7(float w) {
    Target result;
    if (w < 7.31057) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, 0.0, -2.73576, -1.23109);
    } else if (w >= 7.31057 && w < 8.88137) {
        // Trajectory 2: 90 degree arc of circumference
        arco(w - 7.31057 + 2.71874, &result, -21.23109, -6.26424 , 3.0, true);
    } else if (w >= 8.88137 && w < 13.88137) {
        // Trajectory 3: 15 meter straight line
        recta(w - 8.88137, &result, -23.96686, -7.49533, -1.23109, 2.73576);
    } else if (w >= 13.88137 && w < 15.45217) {
        // Trajectory 4: 90 degree arc of circumference
        arco(w - 13.88137 - 1.99365, &result, -27.38656, 7.41459 , 3.0, true);
    } else if (w >= 15.45217) {
        // Trajectory 5: 15 meter straight line
        recta(w - 15.45217, &result, -28.61766, 10.15035, 2.73576, 1.23109);
    } 
    return result;
}

// Quarter circle of radius 4
Target curva8(float w) {
    Target result;
    if (w < 4.69707) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, 0.0, -3.61927, -1.70319);
    } else if (w >= 4.69707 && w < 6.26787) {
        // Trajectory 2: 90 degree arc of circumference
        arco(w - 4.69707 + 2.70175, &result, -18.70319, -4.38073 , 4.0, true);
    } else if (w >= 6.26787 && w < 10.01787) {
        // Trajectory 3: 15 meter straight line
        recta(w - 6.26787, &result, -22.32246, -6.08391, -1.70319, 3.61927);
    } else if (w >= 10.01787 && w < 11.58867) {
        // Trajectory 4: 90 degree arc of circumference
        arco(w - 10.01787 - 2.01064, &result, -25.09014, 9.19156 , 4.0, true);
    } else if (w >= 11.58867) {
        // Trajectory 5: 15 meter straight line
        recta(w - 11.58867, &result, -26.79333, 12.81083, 3.61927, 1.70319);
    } 
    return result;
}

// Quarter circle of radius 5
Target curva9(float w) {
    Target result;
    if (w < 3.31059) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, 0.0, -4.53092, -2.11443);
    } else if (w >= 3.31059 && w < 4.88139) {
        // Trajectory 2: 90 degree arc of circumference
        arco(w - 3.31059 + 2.70497, &result, -17.11443, -2.46908 , 5.0, true);
    } else if (w >= 4.88139 && w < 7.48139) {
        // Trajectory 3: 13 meter straight line
        recta(w - 4.88139, &result, -21.64534, -4.58351, -2.11443, 4.53092);
    } else if (w >= 7.48139 && w < 9.05218) {
        // Trajectory 4: 90 degree arc of circumference
        arco(w - 7.48139 - 2.00742, &result, -22.61194, 9.3113 , 5.0, true);
    } else if (w >= 9.05218) {
        // Trajectory 5: 13 meter straight line
        recta(w - 9.05218, &result, -24.72637, 13.84221, 4.53092, 2.11443);
    } 
    return result;
}


//Complete circuit with arcs of circumference of radius 4
Target curva10(float w) {
    Target result;
    if (w < 4.272) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, 0.0, -3.74532, -1.40449);
    } else if (w >= 4.272 && w < 5.8428) {
        // Trajectory 2: 90 degree arc of circumference
        arco(w - 4.272 + 2.78282, &result, -17.40449, -2.25468, 4.0, true);
    } else if (w >= 5.8428 && w < 9.0928) {
        // Trajectory 3: 13 meter straight line
        recta(w - 5.8428, &result, -21.14981, -3.65918, -1.40449, 3.74532);
    } else if (w >= 9.0928 && w < 10.31453) {
        // Trajectory 4: 70 degree arc of circumference
        arco(w - 9.0928 - 1.92957, &result, -21.9691, 9.9176, 4.0, true);
    } else if (w >= 10.31453 && w < 17.81453) {
        // Trajectory 5: 30 meter straight line
        recta(w - 10.31453, &result, -24.56986, 12.95668, 3.03908, 2.60077);
    } else if (w >= 17.81453 && w < 18.33213) {
        // Trajectory 6: 30 degree arc of circumference
        arco(w - 17.81453 - 0.707836, &result, 0.82401, 29.42334, 4.0, true);
    } else if (w >= 18.33213 && w < 22.08213) {
        // Trajectory 7: 15 meter straight line
        recta(w - 18.33213, &result, 0.09122, 33.35565, 3.93230, 0.732789);
    } else if (w >= 22.08213 && w < 22.60573) {
        // Trajectory 8: 30 degree arc of circumference
        arco(w - 22.08213 - 0.18424, &result, 15.57015, 32.1713, 4.0, true);
    } else if (w >= 22.60573 && w < 25.10573) {
        // Trajectory 9: 10 meter straight line
        recta(w - 22.60573, &result, 16.90169, 35.94317, 3.77187, -1.33154);
    } else if (w >= 25.10573 && w < 26.67653) {
        // Trajectory 10: 90 degree arc of circumference
        arco(w - 25.10573 + 0.33936, &result, 24.99983, 28.84245, 4.0, true);
    } else if (w >= 26.67653 && w < 29.17653) {
        // Trajectory 11: 10 meter straight line
        recta(w - 26.67653, &result, 28.7717, 27.510916, -1.33154, -3.77187);
    } else if (w >= 29.17653 && w < 31.09639) {
        // Trajectory 12: 110 degree arc of circumference
        arco(w - 29.17653 + 1.91016, &result, 21.67098, 19.41278, 4.0, true);
    } else if (w >= 31.09639 && w < 37.34639) {
        // Trajectory 13: 25 meter straight line
        recta(w - 31.09639, &result, 19.12969, 16.32379, -3.08898, 2.54129);
    } else if (w >= 37.34639 && w < 38.91718) {
        // Trajectory 14: 90 degree arc of circumference
        arco(w - 37.34639 + 2.45317, &result, -2.71776, 29.117886, 4.0, false);
    } else if (w >= 38.91718 && w < 47.66718) {
        // Trajectory 15: 35 meter straight line
        recta(w - 38.91718, &result, -5.80675, 31.65918, -2.54129, -3.08898);
    } else if (w >= 47.66718 && w < 49.76157) {
        // Trajectory 16: 120 degree arc of circumference
        arco(w - 47.66718 - 2.25922, &result, -24.95407, 2.08926, 4.0, false);
    } else if (w >= 49.76157) {
        // Trajectory 17: 15 meter straight line
        recta(w - 49.76157, &result, -25.6104, -1.85652, 3.94579, -0.65633);
    }
    return result;
}