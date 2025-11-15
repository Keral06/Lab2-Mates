#include "Quat.hpp"

Quat Quat::Normalized() const
{

	Quat QNormalizado;
    //TODO
	double norma = std::sqrt(s * s + x * x + y * y + z * z);
  
	QNormalizado.s = s /norma;
	QNormalizado.x = x / norma;
	QNormalizado.y = y / norma;
	QNormalizado.z = z / norma;

     

    return QNormalizado;
}

Quat Quat::Multiply(const Quat& b, OpsCounter* op) const
{

    //la formula para multiplicar quaterniones es:
	// s = s1s2 - x1x2 - y1y2 - z1z2
	// x=  s1s2 + x1s2 + y1z2 - z1y2
	// y= s1y2 - x1z2 + y1s2 + z1x2
	// z= s1z2 + x1y2 - y1x2 + z1s2
    Quat Solucion;
	Solucion.s = s * b.s - x * b.x - y * b.y - z * b.z;
	Solucion.x = s * b.x + x * b.s + y * b.z - z * b.y;
	Solucion.y = s * b.y - x * b.z + y * b.s + z * b.x;
	Solucion.z = s * b.z + x * b.y - y * b.x + z * b.s;
    return Solucion;
}

Vec3 Quat::Rotate(const Vec3& v, OpsCounter* op) const
{
	// un quaternion es (s,x,y,z) y su parte de vector ((x,y,z)
	// 
    //TODO

	// 1. t=2q×v
	Vec3 t;
	t.x = 2 *(y * v.z - z * v.y);
	t.y = 2*(z * v.x - x * v.z);
	t.z = 2*(x * v.y - y * v.x);

	//aqui hago el qxt
	Vec3 qxt;
	qxt.x = y * t.z - z * t.y;
	qxt.y = z * t.x - x * t.z;
	qxt.z = x * t.y - y * t.x;
	// 2. v′=v+st+q×t
	Vec3 Rotada;
	Rotada.x = v.x + s * t.x + qxt.x;
	Rotada.y = v.y + s * t.y + qxt.y;
	Rotada.z = v.z + s * t.z + qxt.z;

    return Rotada;
}

Quat Quat::FromMatrix3x3(const Matrix3x3& R)
{
	Quat answer;
    //TODO
	//primero busco el trace
	double trace = R.Trace();
	//si el trace es mayor que 0
	if (trace > 0) {
		double S = 2.0 * std::sqrt(trace + 1.0)/4;

		answer.s = S / 4;
		answer.x = R.At(2, 1) - R.At(1, 2) / S;
		answer.y = R.At(0, 2) - R.At(2, 0) / S;
		answer.z = R.At(1, 0) - R.At(0, 1) / S;
		return answer;
	
	
	}
	//si R00 es el mas grande
	if(R.At(0,0) > R.At(1,1) && R.At(0,0) > R.At(2,2)){
		double S = 2.0 * std::sqrt(1.0 + R.At(0,0) - R.At(1,1) - R.At(2,2));
		answer.s = (R.At(2,1) - R.At(1,2)) / S;
		answer.x = S / 4;
		answer.y = (R.At(0,1) + R.At(1,0)) / S;
		answer.z = (R.At(0,2) + R.At(2,0)) / S;
		return answer;
	}
	//si R11 es el mas grande
	if(R.At(1,1) > R.At(0,0) && R.At(1,1) > R.At(2,2)){
		double S = 2.0 * std::sqrt(1.0 + R.At(1,1) - R.At(0,0) - R.At(2,2));
		answer.s = (R.At(0,2) - R.At(2,0)) / S;
		answer.x = (R.At(0,1) + R.At(1,0)) / S;
		answer.y = S / 4;
		answer.z = (R.At(1,2) + R.At(2,1)) / S;
		return answer;
	}
	//o si R22 es el mas grande
	else {
		double S = 2.0 * std::sqrt(1.0 + R.At(2,2) - R.At(0,0) - R.At(1,1));
		answer.s = (R.At(1,0) - R.At(0,1)) / S;
		answer.x = (R.At(0,2) + R.At(2,0)) / S;
		answer.y = (R.At(1,2) + R.At(2,1)) / S;
		answer.z = S / 4;
		return answer;
	}
   
}

Matrix3x3 Quat::ToMatrix3x3() const
{
    //TODO
	Matrix3x3 R;
	//hay una frmula que se puede seguir en la que te pone que se hace así, así que es como lo he hecho
	R.At(0, 0) = 1 - 2 * (y * y + z * z);
	R.At(0, 1) = 2 * (x * y - z * s);
	R.At(0, 2) = 2 * (x * z + y * s);
	R.At(1, 0) = 2 * (x * y + z * s);
	R.At(1, 1) = 1 - 2 * (x * x + z * z);
	R.At(1, 2) = 2 * (y * z - x * s);
	R.At(2, 0) = 2 * (x * z - y * s);
	R.At(2, 1) = 2 * (y * z + x * s);
	R.At(2, 2) = 1 - 2 * (x * x + y * y);
	
    return R;
}

Quat Quat::FromAxisAngle(const Vec3& u_in, double phi)
{
    //TODO
	// hay una formula vale q= cos(2ϕ​), ux​sin(2ϕ​), uy​sin(2ϕ​), uz​sin(2ϕ​)
	Quat Q;
	Q.x = u_in.x * sin(phi / 2);
	Q.y = u_in.y * sin(phi / 2);
	Q.z = u_in.z * sin(phi / 2);
	return Q;
}

void Quat::ToAxisAngle(Vec3& axis, double& angle) const
{
	//TODO
	//lo contrario que el anterior, hay una formulaa
	// ϕ=2arccos(w)
	// y el eje axis=(x,y,z)/sin(ϕ/2)
	angle = 2 * acos(s)​;
	axis.x = x / sin(angle / 2);
	axis.y = y / sin(angle / 2);
	axis.z = z / sin(angle / 2);

}