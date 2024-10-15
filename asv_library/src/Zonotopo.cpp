// Zonotopo.cpp
#include "asv_library/Zonotopo.h"

// Constructor
Zonotopo::Zonotopo() : orden(0) {
    // Inicializa los miembros de la clase con valores vacíos
    c = VectorXd();
    H = MatrixXd();
}

// Constructor
Zonotopo::Zonotopo(const VectorXd& centro, const MatrixXd& generadores) {
    c = centro;
    H = generadores;
    orden = H.cols();
}

// Método para la reducción de orden
void Zonotopo::reduccionOrden(int s, const MatrixXd& W) {
    MatrixXd H1 = H;
    VectorXd c1 = c;

    int n = c1.size();
    int r = H1.cols();
    MatrixXd Hred = MatrixXd::Zero(n, s);

    // Verifica si el orden solicitado es válido
    if (s <= n) {
        cout << "Error. No se puede reducir el orden a ese valor de s" << endl;
        return;
    } else if (s >= r) {
        cout << "No tiene sentido reducir el orden. Se devuelve el mismo zonotopo" << endl;
        c = c1;
        H = H1;
        return;
    }

    // Creación de vector de normas euclídeas de las columnas de H1
    VectorXd norma = VectorXd::Zero(r);
    for (int j = 0; j < r; ++j) {
        norma(j) = sqrt(H1.col(j).transpose() * W * H1.col(j));
    }

    // Ordena las columnas de H1 en orden decreciente de sus normas
    for (int j = 0; j < r - 1; ++j) {
        int colMayor = mayor(norma, r, j);
        swapColumns(H1, norma, j, colMayor);
    }

    // Cálculo de la matriz Q
    MatrixXd Q = MatrixXd::Zero(n, n);
    for (int i = 0; i < n; ++i) {
        if (i < s - n + 1) continue;
        Q(i, i) = H1.block(i, s - n + 1, 1, r - (s - n + 1)).cwiseAbs().sum();
    }

    // Cálculo de la matriz Hred
    Hred.leftCols(s - n) = H1.leftCols(s - n);
    Hred.rightCols(n) = Q;

    // Actualiza el zonotopo
    c = c1;
    H = Hred;
    orden = Hred.cols();
}

// Método para obtener el índice de la columna con la mayor norma
int Zonotopo::mayor(const VectorXd& normas, int r, int j) {
    int maxIndex = j;
    for (int i = j + 1; i < r; ++i) {
        if (normas(i) > normas(maxIndex)) {
            maxIndex = i;
        }
    }
    return maxIndex;
}

// Método para intercambiar columnas de una matriz y sus respectivas normas
void Zonotopo::swapColumns(MatrixXd& H1, VectorXd& norma, int j, int colMayor) {
    norma.row(j).swap(norma.row(colMayor));
    H1.col(j).swap(H1.col(colMayor));
}

// Método de filtrado
Zonotopo Zonotopo::filteringPsi(const Zonotopo& Z, const Vector<double, 1>& y, const Matrix<double, 1,3>& C, const Matrix<double, 1,1>& R, const Matrix <double, 3,3>& W) {
    int n = Z.c.size();
    MatrixXd I = MatrixXd::Identity(n, n);

    // Cálculo de PZ y PV
    MatrixXd PZ = W * Z.H * Z.H.transpose();
    MatrixXd PV = R * R.transpose();

    // Ganancia local
    MatrixXd L = PZ * C.transpose() * (C * PZ * C.transpose() + PV).inverse();

    // Filtrado
    VectorXd c_fil = Z.c + L * (y - C * Z.c);
    MatrixXd H_fil = (I - L * C) * Z.H;

    // Añadir columnas -L*R a H_fil
    H_fil.conservativeResize(H_fil.rows(), H_fil.cols() + R.cols());
    H_fil.rightCols(R.cols()) = -L * R;

    // Eliminar columnas de ceros
    int num_ceros = 0;
    MatrixXd Hn = MatrixXd::Zero(n, H_fil.cols());
    int l = 0;
    for (int k = 0; k < H_fil.cols(); ++k) {
        if (H_fil.col(k).isZero()) {
            num_ceros++;
        } else {
            Hn.col(l) = H_fil.col(k);
            l++;
        }
    }
    Hn.conservativeResize(n, Hn.cols() - num_ceros);

    // Crear el nuevo zonotopo filtrado
    return Zonotopo(c_fil, Hn);
}

Zonotopo Zonotopo::filteringP(const Zonotopo& Z, const Vector<double, 2>& y, const Matrix<double, 2,6>& C, const Matrix<double, 2,2>& R, const Matrix <double, 6,6>& W) {
    int n = Z.c.size();
    MatrixXd I = MatrixXd::Identity(n, n);

    // Cálculo de PZ y PV
    MatrixXd PZ = W * Z.H * Z.H.transpose();
    MatrixXd PV = R * R.transpose();

    // Ganancia local
    MatrixXd L = PZ * C.transpose() * (C * PZ * C.transpose() + PV).inverse();

    // Filtrado
    VectorXd c_fil = Z.c + L * (y - C * Z.c);
    MatrixXd H_fil = (I - L * C) * Z.H;

    // Añadir columnas -L*R a H_fil
    H_fil.conservativeResize(H_fil.rows(), H_fil.cols() + R.cols());
    H_fil.rightCols(R.cols()) = -L * R;

    // Eliminar columnas de ceros
    int num_ceros = 0;
    MatrixXd Hn = MatrixXd::Zero(n, H_fil.cols());
    int l = 0;
    for (int k = 0; k < H_fil.cols(); ++k) {
        if (H_fil.col(k).isZero()) {
            num_ceros++;
        } else {
            Hn.col(l) = H_fil.col(k);
            l++;
        }
    }
    Hn.conservativeResize(n, Hn.cols() - num_ceros);

    // Crear el nuevo zonotopo filtrado
    return Zonotopo(c_fil, Hn);
}

int Zonotopo::contarCeros(const MatrixXd& H_fil, int col) {
    return H_fil.col(col).isZero() ? 1 : 0;
}

// Definición de la función principal prediction_Y
Zonotopo Zonotopo::prediction_Y(const MatrixXd& Ap, const Zonotopo& Z, 
                                  double min_psi, double max_psi, 
                                  const MatrixXd& Bwp, const MatrixXd& Qp, 
                                  const MatrixXd& Bp_tau) {

    // 1. Obtener rA y mA
    auto [rA, mA] = rad_mid(Ap, min_psi, max_psi);

    // 2. Calcular cx y H1
    VectorXd cx = mA * Z.c + Bp_tau;
    MatrixXd H1 = mA * Z.H;

    // 3. Calcular H2 y H3
    MatrixXd H2 = rs(rA.cwiseAbs() * Z.H);
    MatrixXd H3 = rs(rA.cwiseAbs() * Z.c);

    // 4. Matriz final HZ
    MatrixXd HZ(H1.rows(), H1.cols() + H2.cols() + H3.cols());
    HZ << H1, H2, H3;

    // 5. Eliminar columnas de ceros
    MatrixXd Hn = MatrixXd::Zero(HZ.rows(), HZ.cols());
    int l = 0;
    for (int k = 0; k < HZ.cols(); ++k) {
        if (!HZ.col(k).isZero()) {
            Hn.col(l) = HZ.col(k);
            ++l;
        }
    }
    Hn.conservativeResize(HZ.rows(), l);

    // 6. Perturbaciones
    MatrixXd Hx = MatrixXd(Hn.rows(), Hn.cols() + Bwp.cols());
    Hx << Hn, Bwp * Qp;

    // 7. Crear y retornar el zonotopo resultante
    return Zonotopo(cx, Hx);
}

Zonotopo Zonotopo::prediction2_Y(const Eigen::MatrixXd& Ap, double Ts, const Zonotopo& Z, double min_psi, double max_psi, 
                       const Eigen::MatrixXd& Bwp, const Eigen::MatrixXd& Qp, const Eigen::MatrixXd& Bp_tau, int modo) {
    // Obtener psi medio y rango
    double r_psi = (max_psi - min_psi) / 2;
    double c_psi = (max_psi + min_psi) / 2;

    // 1. R1(psi)
    Eigen::MatrixXd R1(2, 2);
    R1 << -sin(c_psi), -cos(c_psi),
           cos(c_psi), -sin(c_psi);

    R1 = r_psi*R1;

    Eigen::MatrixXd AR1 = Eigen::MatrixXd::Zero(Ap.rows(), Ap.cols());
    AR1.block<2, 2>(0, 2) = Ts * R1;

    // 2. rad[IR0], mid[IR0]
    auto [rad_AIR0, mid_AIR0] = rad_mid_2(Ap, min_psi, max_psi);

    // 2. mid[AIR0]*c, mid[AIR0]*H
    Eigen::VectorXd cx = mid_AIR0 * Z.c + Bp_tau;
    Eigen::MatrixXd H1 = mid_AIR0 * Z.H;

    // 3. rs(rad[AIR0]*H), rs(rad[AIR0]*c)
    Eigen::MatrixXd H2 = rs(rad_AIR0.cwiseAbs() * Z.H);
    Eigen::MatrixXd H3 = rs(rad_AIR0.cwiseAbs() * Z.c);

    // 4. Matriz final
    Eigen::MatrixXd H4 = AR1 * Z.c;
    Eigen::MatrixXd H5 = AR1 * Z.H;

    // Definir HZ según el modo
    Eigen::MatrixXd HZ;
    switch (modo) {
        case 1:
            HZ = Eigen::MatrixXd(H1.rows(), H1.cols() + H2.cols() + H3.cols() + H4.cols() + H5.cols());
            HZ << H1, H2, H3, H4, H5;
            break;
        case 2:
            HZ = Eigen::MatrixXd(H1.rows(), H1.cols() + H4.cols() + H5.cols());
            HZ << H1, H4, H5;
            break;
        case 3:
            HZ = Eigen::MatrixXd(H1.rows(), H1.cols() + H4.cols() + H5.cols());
            HZ << H1, 0 * H4, 0 * H5;
            break;
    }

    // Remover columnas con ceros
    Eigen::MatrixXd Hn = Eigen::MatrixXd::Zero(HZ.rows(), HZ.cols());
    int l = 0;
    for (int k = 0; k < HZ.cols(); ++k) {
        if (HZ.col(k).norm() != 0) {
            Hn.col(l) = HZ.col(k);
            l++;
        }
    }
    Hn.conservativeResize(Hn.rows(), l);

    // 5. Perturbaciones
    Eigen::MatrixXd Hx = Eigen::MatrixXd::Zero(Hn.rows(), Hn.cols() + Qp.cols());
    Hx << Hn, Bwp * Qp;

    // Crear zonotope resultante
    Zonotopo Zp;
    Zp.c = cx;
    Zp.H = Hx;
    
    return Zp;
}

// Definición de la función rad_mid
std::pair<MatrixXd, MatrixXd> Zonotopo::rad_mid(const MatrixXd& Ap, double min_psi, double max_psi) {
    auto [c, s] = max_min_trig(min_psi, max_psi);

    MatrixXd Rmin(2, 2), Rmax(2, 2);
    Rmin << c(0), -s(1), s(0), c(0);
    Rmax << c(1), -s(0), s(1), c(1);

    MatrixXd ARmin = Ap;
    ARmin.block(0, 2, 2, 2) *= Rmin;

    MatrixXd ARmax = Ap;
    ARmax.block(0, 2, 2, 2) *= Rmax;

    MatrixXd mA = (ARmin + ARmax) / 2;
    MatrixXd rA = (ARmax - ARmin) / 2;

    return {rA, mA};
}

// Definición de la función rs
MatrixXd Zonotopo::rs(const MatrixXd& H) {
    MatrixXd M = MatrixXd::Zero(H.rows(), H.rows());
    for (int i = 0; i < H.rows(); ++i) {
        for (int j = 0; j < H.cols(); ++j) {
            M(i, i) += std::abs(H(i, j));
        }
    }
    return M;
}

// Definición de la función max_min_trig
std::pair<MatrixXd, MatrixXd> Zonotopo::max_min_trig(double min_psi, double max_psi) {
    double delta_psi = max_psi - min_psi;

    // Comprobar si el intervalo es demasiado amplio
    if (delta_psi > M_PI / 4) {
        cout << "The interval of the trigonometric functions is not correct" << endl;
        return {MatrixXd::Zero(2, 1), MatrixXd::Zero(2, 1)};
    }

    // Ajustar los ángulos para mover el intervalo a ser positivo y lo más cercano a cero
    while (min_psi > 0) {
        min_psi -= 2 * M_PI;
        max_psi -= 2 * M_PI;
    }
    while (min_psi < 0) {
        min_psi += 2 * M_PI;
        max_psi += 2 * M_PI;
    }

    double c_min, c_max, s_min, s_max;

    // Determinar los límites para el coseno y seno dependiendo del intervalo
    if (min_psi < M_PI / 2 && max_psi > M_PI / 2) {
        c_min = cos(max_psi);
        c_max = cos(min_psi);
        s_min = std::min(sin(min_psi), sin(max_psi));
        s_max = 1;
    } else if (min_psi < M_PI && max_psi > M_PI) {
        c_min = -1;
        c_max = std::max(cos(min_psi), cos(max_psi));
        s_min = sin(max_psi);
        s_max = sin(min_psi);
    } else if (min_psi < 3 * M_PI / 2 && max_psi > 3 * M_PI / 2) {
        c_min = cos(min_psi);
        c_max = cos(max_psi);
        s_min = -1;
        s_max = std::max(sin(min_psi), sin(max_psi));
    } else if (min_psi < 2 * M_PI && max_psi > 2 * M_PI) {
        c_min = std::min(cos(min_psi), cos(max_psi));
        c_max = 1;
        s_min = sin(min_psi);
        s_max = sin(max_psi);
    } else {
        c_min = std::min(cos(min_psi), cos(max_psi));
        c_max = std::max(cos(min_psi), cos(max_psi));
        s_min = std::min(sin(min_psi), sin(max_psi));
        s_max = std::max(sin(min_psi), sin(max_psi));
    }

    // Crear las matrices de salida c y s
    MatrixXd c(2, 1), s(2, 1);
    c << c_min, c_max;
    s << s_min, s_max;
    return {c, s};
}

std::pair<double, double> Zonotopo::int_prod(const Eigen::MatrixXd& a, const std::pair<double, double>& b) {
    // Calcular el mínimo y el máximo del producto de los elementos de los intervalos
    double v_min = std::min({a(0,0) * b.first, a(0,0) * b.second, a(1,0) * b.first, a(1,0) * b.second});
    double v_max = std::max({a(0,0) * b.first, a(0,0) * b.second, a(1,0) * b.first, a(1,0) * b.second});

    // Devolver el resultado como un par {mínimo, máximo}
    return {v_min, v_max};
}

// Implementación de rad_mid en C++
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> Zonotopo::rad_mid_2(const Eigen::MatrixXd& Ap, double min_psi, double max_psi) {
    double r_psi = (max_psi - min_psi) / 2;
    double c_psi = (max_psi + min_psi) / 2;

    // Llamada a max_min_trig
    auto [c, s] = max_min_trig(min_psi, max_psi);

    std::pair<double, double> m ={-0.5, 0.0};

    // Producto intervalar
    std::pair<double, double> vc = int_prod(c, m);
    std::pair<double, double> vs = int_prod(s, m);

    // Matriz C basada en cos(c_psi) y sin(c_psi)
    Eigen::Matrix2d C;
    C << std::cos(c_psi), -std::sin(c_psi),
         std::sin(c_psi), std::cos(c_psi);

    // Cálculo de Rmin y Rmax
    Eigen::Matrix2d Rmin = C + r_psi * r_psi * Eigen::Matrix2d{{vc.first, -vs.second}, {vs.first, vc.first}};
    Eigen::Matrix2d Rmax = C + r_psi * r_psi * Eigen::Matrix2d{{vc.second, -vs.first}, {vs.second, vc.second}};

    // Matrices ARmin y ARmax
    Eigen::MatrixXd ARmin = Ap;
    Eigen::MatrixXd ARmax = Ap;
    ARmin.block<2, 2>(0, 2) = ARmin.block<2, 2>(0, 2) * Rmin;
    ARmax.block<2, 2>(0, 2) = ARmax.block<2, 2>(0, 2) * Rmax;

    // Cálculo de mA y rA
    Eigen::MatrixXd mA = (ARmin + ARmax) / 2;
    Eigen::MatrixXd rA = (ARmax - ARmin) / 2;

    return {rA, mA};
}