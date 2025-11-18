#include "Matrix3x3.hpp"
#include <stdexcept>

#define TOL 1e-6
#define PI 3.14159265358979323846

// ------------------ Vec3 -------------------------

double Vec3::Dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 Vec3::Cross(const Vec3& a, const Vec3& b)
{
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

double Vec3::Norm() const
{
    return std::sqrt(Dot(*this, *this));
}

Vec3 Vec3::Normalize() const
{
    double n = Norm();
    if (n == 0) throw std::invalid_argument("normalize: zero vector");
    return { x / n, y / n, z / n };
}

// ------------------ Matrix3x3 ---------------------

Matrix3x3 Matrix3x3::Identity()
{
    Matrix3x3 I;
    I.At(0, 0) = 1; I.At(1, 1) = 1; I.At(2, 2) = 1;
    return I;
}

Vec3 Matrix3x3::Multiply(const Vec3& x, OpsCounter* op) const
{
    // y = A * x
    Vec3 y;
    y.x = At(0, 0) * x.x + At(0, 1) * x.y + At(0, 2) * x.z;
    y.y = At(1, 0) * x.x + At(1, 1) * x.y + At(1, 2) * x.z;
    y.z = At(2, 0) * x.x + At(2, 1) * x.y + At(2, 2) * x.z;
    if (op) { op->IncMul(9); op->IncAdd(6); }
    return y;
}

Matrix3x3 Matrix3x3::Multiply(const Matrix3x3& B, OpsCounter* op) const
{
    Matrix3x3 C{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) {
                s += At(i, k) * B.At(k, j);
            }
            C.At(i, j) = s;
        }
    }
    if (op) { op->IncMul(27); op->IncAdd(18); }
    return C;
}

double Matrix3x3::Det() const
{
    //TODO
    

    double det_suma = (m[0] * m[4] * m[8]) + (m[1] * m[5] * m[6]) + (m[2] * m[3] * m[7]);
    double det_resta = (m[2] * m[4] * m[6]) + (m[1] * m[3] * m[8]) + (m[5] * m[7] * m[0]);
    double resultat = det_suma - det_resta;
    
    return resultat;
}

Matrix3x3 Matrix3x3::Transposed() const
{
    //TODO
    Matrix3x3 C;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C.At(i, j) = At(j, i);
        }
        
    }
    return C;
}

double Matrix3x3::Trace() const
{
    //TODO
    double trace = At(0, 0) + At(1, 1) + At(2, 2);
    return trace;
}

bool Matrix3x3::IsRotation() const
{
    //TODO
    Matrix3x3 C;
    Matrix3x3 T;
    T = this->Transposed();
    Matrix3x3 TM = T.Multiply(*this);
    Matrix3x3 I = Matrix3x3::Identity();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++){
            if(TM.At(i, j) != I.At(i, j)) {
                return false;
            }
        }
    }

    if (this->Det() != 1.0) {
        return false;
	}
	
    return true;
 

}

Matrix3x3 Matrix3x3::RotationAxisAngle(const Vec3& u_in, double phi)
{
    //TODO
    //R = (first)I cos ϕ + (second)(1 − cos ϕ) uuT + (third)[u]× sin ϕ
    Matrix3x3 C;
	Vec3 eje = u_in.Normalize();
    double u[3] = { eje.x, eje.y, eje.z };
    double K[3][3] = {
    { 0.0, -eje.z,  eje.y },
    {  eje.z, 0.0, -eje.x },
    { -eje.y,  eje.x, 0.0 }
    };
    for (int i = 0; i < 3 ; i++) {
        for (int j = 0; j < 3; j++) {
            double first;
            if (i == j) {
                first = 1 * cos(phi);
            }
            else {
                first = 0.0;
            }
			double second = (1.0 - cos(phi)) * u[i] * u[j];
            double third = sin(phi) * K[i][j];
            C.At(i, j) = first + second + third;
        }
    }

    return C;
}

Vec3 Matrix3x3::Rotate(const Vec3& v, OpsCounter* op) const
{
    //TODO
	Vec3 result = Multiply(v, op);
    return result;
}

void Matrix3x3::ToAxisAngle(Vec3& axis, double& angle) const
{
    

	double trace = Trace();
	angle = acos((trace - 1) / 2.0);
	//pone que si es 180 o 0 es un caso especial pero no diu res de com fer-ho la presentacion
    double s_angle = sin(angle);

    if (s_angle == 0.0)
    {
        if (angle == 0.0) { // caso 0 grados
            angle = 0.0;
            axis.x = 1.0; axis.y = 0.0; axis.z = 0.0;
        }
        else { // caso 180 grados
            angle = PI;
            double xx = (At(0, 0) + 1.0) / 2.0;
            double yy = (At(1, 1) + 1.0) / 2.0;
            double zz = (At(2, 2) + 1.0) / 2.0;
            double xy = (At(0, 1) + At(1, 0)) / 4.0;
            double xz = (At(0, 2) + At(2, 0)) / 4.0;
            double yz = (At(1, 2) + At(2, 1)) / 4.0;

            if (xx > yy && xx > zz) {
                axis.x = std::sqrt(xx); axis.y = xy / axis.x; axis.z = xz / axis.x;
            }
            else if (yy > zz) {
                axis.y = std::sqrt(yy); axis.x = xy / axis.y; axis.z = yz / axis.y;
            }
            else {
                axis.z = std::sqrt(zz); axis.x = xz / axis.z; axis.y = yz / axis.z;
            }
        }
    }
    else
    {
        double div = 2.0 * s_angle;
        axis.x = (At(2, 1) - At(1, 2)) / div;
        axis.y = (At(0, 2) - At(2, 0)) / div;
        axis.z = (At(1, 0) - At(0, 1)) / div;
        axis = axis.Normalize();
    }
    //TODO

}

Matrix3x3 Matrix3x3::FromEulerZYX(double yaw, double pitch, double roll)
{
    //yaw = rotación sobre Z
	OpsCounter* op = nullptr;
    //pitch = rotación sobre Y

    //roll = rotación sobre X
    // 
	Matrix3x3 Rx; // rotación sobre X 
	Rx.m[0] = 1;
    Rx.m[1] = 0;
    Rx.m[2] = 0;
	Rx.m[3] = 0;   
	Rx.m[4] = cos(roll);
    Rx.m[5] = -sin(roll);
	Rx.m[6] = 0;
	Rx.m[7] = sin(roll);
	Rx.m[8] = cos(roll);
	Matrix3x3 Ry; // rotación sobre Y
	Ry.m[0] = cos(pitch);
	Ry.m[1] = 0;
	Ry.m[2] = sin(pitch);
	Ry.m[3] = 0;
	Ry.m[4] = 1;
	Ry.m[5] = 0;
	Ry.m[6] = -sin(pitch);
	Ry.m[7] = 0;
	Ry.m[8] = cos(pitch);
	Matrix3x3 Rz; // rotación sobre Z
	Rz.m[0] = cos(yaw);
	Rz.m[1] = -sin(yaw);
	Rz.m[2] = 0;
	Rz.m[3] = sin(yaw);
	Rz.m[4] = cos(yaw);
	Rz.m[5] = 0;
	Rz.m[6] = 0;
	Rz.m[7] = 0;
	Rz.m[8] = 1;
    Matrix3x3 R;
    R = Rz.Multiply(Ry, op);


    //TODO
    return R.Multiply(Rx,op);
}

void Matrix3x3::ToEulerZYX(double& yaw, double& pitch, double& roll) const
{

	
    double sp = -At(2, 0);

    if (sp >= 1.0) {
        pitch = PI / 2.0; // 90 grados
    }
    else if (sp <= -1.0) {
        pitch = -PI / 2.0; // -90 grados
    }
    else {
        pitch = asin(sp);
    }

    double cp = cos(pitch);

    if (cp != 0)
    {
        yaw = atan2(At(1, 0), At(0, 0));

        roll = atan2(At(2, 1), At(2, 2));
    }
    else
    {
        roll = 0.0;

        yaw = atan2(-At(0, 1), At(1, 1));
    }
    //TODO
}