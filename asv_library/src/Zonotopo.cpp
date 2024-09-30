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
    MatrixXd c(2, 1), s(2, 1);
    c << cos(min_psi), cos(max_psi);
    s << sin(min_psi), sin(max_psi);

    return {c, s};
}