/*
Copyright (c) 2007, Markus Trenkwalder

All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation 
  and/or other materials provided with the distribution.

* Neither the name of the library's copyright owner nor the names of its 
  contributors may be used to endorse or promote products derived from this 
  software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

//#include <cmath>

// "minor" can be defined from GCC and can cause problems
#undef minor

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace vmath {

//using std::sin;
//using std::cos;
//using std::acos;
//using std::sqrt;

template <typename T>
inline T rsqrt(T x)
{
	return T(1) / sqrt(x);
}

template <typename T>
inline T inv(T x)
{
	return T(1) / x;
}

namespace detail {
	// This function is used heavily in this library. Here is a generic 
	// implementation for it. If you can provide a faster one for your specific
	// types this can speed up things considerably.
	template <typename T>
	inline T multiply_accumulate(int count, const T *a, const T *b)
	{
		T result = T(0);
		for (int i = 0; i < count; ++i)
			result += a[i] * b[i];
		return result;
	}
}

#define MOP_M_CLASS_TEMPLATE(CLASS, OP, COUNT) \
	CLASS & operator OP (const CLASS& rhs)  \
	{ \
		for (int i = 0; i < (COUNT); ++i )  \
			(*this)[i] OP rhs[i]; \
		return *this; \
	}

#define MOP_M_TYPE_TEMPLATE(CLASS, OP, COUNT) \
	CLASS & operator OP (const T & rhs) \
	{ \
		for (int i = 0; i < (COUNT); ++i ) \
			(*this)[i] OP rhs; \
		return *this; \
	}

#define MOP_COMP_TEMPLATE(CLASS, COUNT) \
	bool operator == (const CLASS & rhs) \
	{ \
		bool result = true; \
		for (int i = 0; i < (COUNT); ++i) \
			result = result && (*this)[i] == rhs[i]; \
		return result; \
	} \
	bool operator != (const CLASS & rhs) \
	{ return !((*this) == rhs); }

#define MOP_G_UMINUS_TEMPLATE(CLASS, COUNT) \
	CLASS operator - () const \
	{ \
		CLASS result; \
		for (int i = 0; i < (COUNT); ++i) \
			result[i] = -(*this)[i]; \
		return result; \
	}

#define COMMON_OPERATORS(CLASS, COUNT) \
	MOP_M_CLASS_TEMPLATE(CLASS, +=, COUNT) \
	MOP_M_CLASS_TEMPLATE(CLASS, -=, COUNT) \
	/*no *= as this is not the same for vectors and matrices */ \
	MOP_M_CLASS_TEMPLATE(CLASS, /=, COUNT) \
	MOP_M_TYPE_TEMPLATE(CLASS, +=, COUNT) \
	MOP_M_TYPE_TEMPLATE(CLASS, -=, COUNT) \
	MOP_M_TYPE_TEMPLATE(CLASS, *=, COUNT) \
	MOP_M_TYPE_TEMPLATE(CLASS, /=, COUNT) \
	MOP_G_UMINUS_TEMPLATE(CLASS, COUNT) \
	MOP_COMP_TEMPLATE(CLASS, COUNT)

#define VECTOR_COMMON(CLASS, COUNT) \
	COMMON_OPERATORS(CLASS, COUNT) \
	MOP_M_CLASS_TEMPLATE(CLASS, *=, COUNT) \
	operator const T* () const { return &x; } \
	operator T* () { return &x; }

#define FOP_G_SOURCE_TEMPLATE(OP, CLASS) \
	{ CLASS<T> r = lhs; r OP##= rhs; return r; }

#define FOP_G_CLASS_TEMPLATE(OP, CLASS) \
	template <typename T> \
	inline CLASS<T> operator OP (const CLASS<T> &lhs, const CLASS<T> &rhs) \
	FOP_G_SOURCE_TEMPLATE(OP, CLASS)

#define FOP_G_TYPE_TEMPLATE(OP, CLASS) \
	template <typename T> \
	inline CLASS<T> operator OP (const CLASS<T> &lhs, const T &rhs) \
	FOP_G_SOURCE_TEMPLATE(OP, CLASS)

// forward declarations
template <typename T> struct vec2;
template <typename T> struct vec3;
template <typename T> struct vec4;
template <typename T> struct mat2;
template <typename T> struct mat3;
template <typename T> struct mat4;
template <typename T> struct quat;

#define FREE_MODIFYING_OPERATORS(CLASS) \
	FOP_G_CLASS_TEMPLATE(+, CLASS) \
	FOP_G_CLASS_TEMPLATE(-, CLASS) \
	FOP_G_CLASS_TEMPLATE(*, CLASS) \
	FOP_G_CLASS_TEMPLATE(/, CLASS) \
	FOP_G_TYPE_TEMPLATE(+, CLASS) \
	FOP_G_TYPE_TEMPLATE(-, CLASS) \
	FOP_G_TYPE_TEMPLATE(*, CLASS) \
	FOP_G_TYPE_TEMPLATE(/, CLASS)

FREE_MODIFYING_OPERATORS(vec2)
FREE_MODIFYING_OPERATORS(vec3)
FREE_MODIFYING_OPERATORS(vec4)
FREE_MODIFYING_OPERATORS(mat2)
FREE_MODIFYING_OPERATORS(mat3)
FREE_MODIFYING_OPERATORS(mat4)
FREE_MODIFYING_OPERATORS(quat)

#define FREE_OPERATORS(CLASS) \
	template <typename T> \
	inline CLASS<T> operator + (const T& a, const CLASS<T>& b)  \
	{ CLASS<T> r = b; r += a; return r; } \
	\
	template <typename T> \
	inline CLASS<T> operator * (const T& a, const CLASS<T>& b)  \
	{ CLASS<T> r = b; r *= a; return r; } \
	\
	template <typename T> \
	inline CLASS<T> operator - (const T& a, const CLASS<T>& b)  \
	{ return -b + a; } \
	\
	template <typename T> \
	inline CLASS<T> operator / (const T& a, const CLASS<T>& b)  \
	{ CLASS<T> r(a); r /= b; return r; }

FREE_OPERATORS(vec2)
FREE_OPERATORS(vec3)
FREE_OPERATORS(vec4)
FREE_OPERATORS(mat2)
FREE_OPERATORS(mat3)
FREE_OPERATORS(mat4)
FREE_OPERATORS(quat)

template <typename T>
struct vec2 {
	T x, y;
	
	vec2() {};
	explicit vec2(const T i) : x(i), y(i) {}
	explicit vec2(const T ix, const T iy) : x(ix), y(iy) {}
	explicit vec2(const vec3<T>& v);
	explicit vec2(const vec4<T>& v);

	VECTOR_COMMON(vec2, 2)
};

template <typename T> 
struct vec3 {
	T x, y, z;
	
	vec3() {};
	explicit vec3(const T i) : x(i), y(i), z(i) {}
	explicit vec3(const T ix, const T iy, const T iz) : x(ix), y(iy), z(iz) {}
	explicit vec3(const vec2<T>& xy, const T iz) : x(xy.x), y(xy.y), z(iz) {}
	explicit vec3(const T ix, const vec2<T>& yz) : x(ix), y(yz.y), z(yz.z) {}
	explicit vec3(const vec4<T>& v);

	VECTOR_COMMON(vec3, 3)
};

template <typename T> 
struct vec4 {
	T x, y, z, w;
	
	vec4() {};
	explicit vec4(const T i) : x(i), y(i), z(i), w(i) {}
	explicit vec4(const T ix, const T iy, const T iz, const T iw) : x(ix), y(iy), z(iz), w(iw) {}
	explicit vec4(const vec3<T>& xyz,const T iw) : x(xyz.x), y(xyz.y), z(xyz.z), w(iw) {}
	explicit vec4(const T ix, const vec3<T>& yzw) : x(ix), y(yzw.x), z(yzw.y), w(yzw.z) {}
	explicit vec4(const vec2<T>& xy, const vec2<T>& zw) : x(xy.x), y(xy.y), z(zw.x), w(zw.y) {}

	VECTOR_COMMON(vec4, 4)
};

// additional constructors that omit the last element
template <typename T> inline vec2<T>::vec2(const vec3<T>& v) : x(v.x), y(v.y) {}
template <typename T> inline vec2<T>::vec2(const vec4<T>& v) : x(v.x), y(v.y) {}
template <typename T> inline vec3<T>::vec3(const vec4<T>& v) : x(v.x), y(v.y), z(v.z) {}

#define VEC_QUAT_FUNC_TEMPLATE(CLASS, COUNT) \
	template <typename T> \
	inline T dot(const CLASS & u, const CLASS & v) \
	{ \
		const T *a = u; \
		const T *b = v; \
		using namespace detail; \
		return multiply_accumulate(COUNT, a, b); \
	} \
	template <typename T> \
	inline T length(const CLASS & v) \
	{ \
		return sqrt(dot(v, v)); \
	} \
	template <typename T> inline CLASS normalize(const CLASS & v) \
	{ \
		return v * rsqrt(dot(v, v)); \
	} \
	template <typename T> inline CLASS lerp(const CLASS & u, const CLASS & v, const T x) \
	{ \
		return u * (T(1) - x) + v * x; \
	}

VEC_QUAT_FUNC_TEMPLATE(vec2<T>, 2)
VEC_QUAT_FUNC_TEMPLATE(vec3<T>, 3)
VEC_QUAT_FUNC_TEMPLATE(vec4<T>, 4)
VEC_QUAT_FUNC_TEMPLATE(quat<T>, 4)

#define VEC_FUNC_TEMPLATE(CLASS) \
	template <typename T> inline CLASS reflect(const CLASS & I, const CLASS & N) \
	{ \
		return I - T(2) * dot(N, I) * N; \
	} \
	template <typename T> inline CLASS refract(const CLASS & I, const CLASS & N, T eta) \
	{ \
		const T d = dot(N, I); \
		const T k = T(1) - eta * eta * (T(1) - d * d); \
		if ( k < T(0) ) \
			return CLASS(T(0)); \
		else \
			return eta * I - (eta * d + static_cast<T>(sqrt(k))) * N; \
	} 

VEC_FUNC_TEMPLATE(vec2<T>)
VEC_FUNC_TEMPLATE(vec3<T>)
VEC_FUNC_TEMPLATE(vec4<T>)

template <typename T> inline T lerp(const T & u, const T & v, const T x) 
{ 
	return dot(vec2<T>(u, v), vec2<T>((T(1) - x), x));
}

template <typename T> inline vec3<T> cross(const vec3<T>& u, const vec3<T>& v)
{
	return vec3<T>(
		dot(vec2<T>(u.y, -v.y), vec2<T>(v.z, u.z)),
		dot(vec2<T>(u.z, -v.z), vec2<T>(v.x, u.x)),
		dot(vec2<T>(u.x, -v.x), vec2<T>(v.y, u.y)));
}


#define MATRIX_COL4(SRC, C) \
	vec4<T>(SRC.elem[0][C], SRC.elem[1][C], SRC.elem[2][C], SRC.elem[3][C])

#define MATRIX_ROW4(SRC, R) \
	vec4<T>(SRC.elem[R][0], SRC.elem[R][1], SRC.elem[R][2], SRC.elem[R][3])

#define MATRIX_COL3(SRC, C) \
	vec3<T>(SRC.elem[0][C], SRC.elem[1][C], SRC.elem[2][C])

#define MATRIX_ROW3(SRC, R) \
	vec3<T>(SRC.elem[R][0], SRC.elem[R][1], SRC.elem[R][2])

#define MATRIX_COL2(SRC, C) \
	vec2<T>(SRC.elem[0][C], SRC.elem[1][C])

#define MATRIX_ROW2(SRC, R) \
	vec2<T>(SRC.elem[R][0], SRC.elem[R][1])

#define MOP_M_MATRIX_MULTIPLY(CLASS, SIZE) \
	CLASS & operator *= (const CLASS & rhs) \
	{ \
		CLASS result; \
		for (int r = 0; r < SIZE; ++r) \
		for (int c = 0; c < SIZE; ++c) \
			result.elem[r][c] = dot( \
				MATRIX_ROW ## SIZE((*this), r), \
				MATRIX_COL ## SIZE(rhs, c)); \
		return (*this) = result; \
	}

#define MATRIX_CONSTRUCTOR_FROM_T(CLASS, SIZE) \
	explicit CLASS(const T v) \
	{ \
		for (int r = 0; r < SIZE; ++r) \
		for (int c = 0; c < SIZE; ++c) \
			if (r == c) elem[r][c] = v; \
			else elem[r][c] = T(0); \
	}

#define MATRIX_CONSTRUCTOR_FROM_LOWER(CLASS1, CLASS2, SIZE1, SIZE2) \
	explicit CLASS1(const CLASS2<T>& m) \
	{ \
		for (int r = 0; r < SIZE1; ++r) \
		for (int c = 0; c < SIZE1; ++c) \
			if (r < SIZE2 && c < SIZE2) elem[r][c] = m.elem[r][c]; \
			else elem[r][c] = r == c ? T(1) : T(0); \
	}

#define MATRIX_COMMON(CLASS, SIZE) \
	COMMON_OPERATORS(CLASS, SIZE*SIZE) \
	MOP_M_MATRIX_MULTIPLY(CLASS, SIZE) \
	MATRIX_CONSTRUCTOR_FROM_T(CLASS, SIZE) \
	operator const T* () const { return (const T*) elem; } \
	operator T* () { return (T*) elem; }

template <typename T> struct mat2;
template <typename T> struct mat3;
template <typename T> struct mat4;

template <typename T> 
struct mat2  {
	T elem[2][2];
	
	mat2() {}
	
	explicit mat2(
		const T m00, const T m01,
		const T m10, const T m11)
	{
		elem[0][0] = m00; elem[0][1] = m01;
		elem[1][0] = m10; elem[1][1] = m11;
	}

	explicit mat2(const vec2<T>& v0, const vec2<T>& v1)
	{
		elem[0][0] = v0[0];
		elem[1][0] = v0[1];
		elem[0][1] = v1[0];
		elem[1][1] = v1[1];
	}

	explicit mat2(const mat3<T>& m);

	MATRIX_COMMON(mat2, 2)
};

template <typename T> 
struct mat3 {
	T elem[3][3];
	
	mat3() {}
	
	explicit mat3(
		const T m00, const T m01, const T m02,
		const T m10, const T m11, const T m12,
		const T m20, const T m21, const T m22)
	{
		elem[0][0] = m00; elem[0][1] = m01; elem[0][2] = m02;
		elem[1][0] = m10; elem[1][1] = m11; elem[1][2] = m12;
		elem[2][0] = m20; elem[2][1] = m21; elem[2][2] = m22;
	}
	
	explicit mat3(const vec3<T>& v0, const vec3<T>& v1, const vec3<T>& v2)
	{
		elem[0][0] = v0[0];
		elem[1][0] = v0[1];
		elem[2][0] = v0[2];
		elem[0][1] = v1[0];
		elem[1][1] = v1[1];
		elem[2][1] = v1[2];
		elem[0][2] = v2[0];
		elem[1][2] = v2[1];
		elem[2][2] = v2[2];
	}

	explicit mat3(const mat4<T>& m);

	MATRIX_CONSTRUCTOR_FROM_LOWER(mat3, mat2, 3, 2)
	MATRIX_COMMON(mat3, 3)
};

template <typename T> 
struct mat4 {
	T elem[4][4];
	
	mat4() {}

	explicit mat4(
		const T m00, const T m01, const T m02, const T m03,
		const T m10, const T m11, const T m12, const T m13,
		const T m20, const T m21, const T m22, const T m23,
		const T m30, const T m31, const T m32, const T m33)
	{
		elem[0][0] = m00; elem[0][1] = m01; elem[0][2] = m02; elem[0][3] = m03;
		elem[1][0] = m10; elem[1][1] = m11; elem[1][2] = m12; elem[1][3] = m13;
		elem[2][0] = m20; elem[2][1] = m21; elem[2][2] = m22; elem[2][3] = m23;
		elem[3][0] = m30; elem[3][1] = m31; elem[3][2] = m32; elem[3][3] = m33;
	}

	explicit mat4(const vec4<T>& v0, const vec4<T>& v1, const vec4<T>& v2, const vec4<T>& v3)
	{
		elem[0][0] = v0[0];
		elem[1][0] = v0[1];
		elem[2][0] = v0[2];
		elem[3][0] = v0[3];
		elem[0][1] = v1[0];
		elem[1][1] = v1[1];
		elem[2][1] = v1[2];
		elem[3][1] = v1[3];
		elem[0][2] = v2[0];
		elem[1][2] = v2[1];
		elem[2][2] = v2[2];
		elem[3][2] = v2[3];
		elem[0][3] = v3[0];
		elem[1][3] = v3[1];
		elem[2][3] = v3[2];
		elem[3][3] = v3[3];
	}

	MATRIX_CONSTRUCTOR_FROM_LOWER(mat4, mat3, 4, 3)
	MATRIX_COMMON(mat4, 4)
};

#define MATRIX_CONSTRUCTOR_FROM_HIGHER(CLASS1, CLASS2, SIZE) \
	template <typename T> \
	inline CLASS1<T>::CLASS1(const CLASS2<T>& m) \
	{ \
		for (int r = 0; r < SIZE; ++r) \
		for (int c = 0; c < SIZE; ++c) \
			elem[r][c] = m.elem[r][c]; \
	}

MATRIX_CONSTRUCTOR_FROM_HIGHER(mat2, mat3, 2)
MATRIX_CONSTRUCTOR_FROM_HIGHER(mat3, mat4, 3)

#define MAT_FUNC_TEMPLATE(CLASS, SIZE) \
	template <typename T>  \
	inline CLASS transpose(const CLASS & m) \
	{ \
		CLASS result; \
		for (int r = 0; r < SIZE; ++r) \
		for (int c = 0; c < SIZE; ++c) \
			result.elem[r][c] = m.elem[c][r]; \
		return result; \
	} \
	template <typename T>  \
	inline CLASS identity ## SIZE() \
	{ \
		CLASS result; \
		for (int r = 0; r < SIZE; ++r) \
		for (int c = 0; c < SIZE; ++c) \
			result.elem[r][c] = r == c ? T(1) : T(0); \
		return result; \
	} \
	template <typename T> \
	inline T trace(const CLASS & m) \
	{ \
		T result = T(0); \
		for (int i = 0; i < SIZE; ++i) \
			result += m.elem[i][i]; \
		return result; \
	}

MAT_FUNC_TEMPLATE(mat2<T>, 2)
MAT_FUNC_TEMPLATE(mat3<T>, 3)
MAT_FUNC_TEMPLATE(mat4<T>, 4)

#define MAT_FUNC_MINOR_TEMPLATE(CLASS1, CLASS2, SIZE) \
	template <typename T>  \
	inline CLASS2 minor(const CLASS1 & m, int _r = SIZE, int _c = SIZE) { \
		CLASS2 result; \
		for (int r = 0; r < SIZE - 1; ++r) \
		for (int c = 0; c < SIZE - 1; ++c) { \
			int rs = r >= _r ? 1 : 0; \
			int cs = c >= _c ? 1 : 0; \
			result.elem[r][c] = m.elem[r + rs][c + cs]; \
		} \
		return result; \
	}

MAT_FUNC_MINOR_TEMPLATE(mat3<T>, mat2<T>, 3)
MAT_FUNC_MINOR_TEMPLATE(mat4<T>, mat3<T>, 4)

template <typename T>
inline T det(const mat2<T>& m)
{
	return dot(
		vec2<T>(m.elem[0][0], -m.elem[0][1]), 
		vec2<T>(m.elem[1][1], m.elem[1][0]));
}

template <typename T>
inline T det(const mat3<T>& m)
{
	return dot(cross(MATRIX_COL3(m, 0), MATRIX_COL3(m, 1)), MATRIX_COL3(m, 2));
}

template <typename T>
inline T det(const mat4<T>& m)
{
	vec4<T> b;
	for (int i = 0; i < 4; ++i)
		b[i] = (i & 1 ? -1 : 1) * det(minor(m, 0, i));
	return dot(MATRIX_ROW4(m, 0), b);
}

#define MAT_ADJOINT_TEMPLATE(CLASS, SIZE) \
	template <typename T> \
	inline CLASS adjoint(const CLASS & m) \
	{ \
		CLASS result; \
		for (int r = 0; r < SIZE; ++r) \
		for (int c = 0; c < SIZE; ++c) \
			result.elem[r][c] = ((r + c) & 1 ? -1 : 1) * det(minor(m, c, r)); \
		return result; \
	}

MAT_ADJOINT_TEMPLATE(mat3<T>, 3)
MAT_ADJOINT_TEMPLATE(mat4<T>, 4)

template <typename T>
inline mat2<T> adjoint(const mat2<T> & m)
{
	return mat2<T>(
		 m.elem[1][1], -m.elem[0][1],
		-m.elem[1][0],  m.elem[0][0]
	);
}

#define MAT_INVERSE_TEMPLATE(CLASS) \
	template <typename T> \
	inline CLASS inverse(const CLASS & m) \
	{ \
		return adjoint(m) * inv(det(m)); \
	}

MAT_INVERSE_TEMPLATE(mat2<T>)
MAT_INVERSE_TEMPLATE(mat3<T>)
MAT_INVERSE_TEMPLATE(mat4<T>)

#define MAT_VEC_FUNCS_TEMPLATE(MATCLASS, VECCLASS, SIZE) \
	template <typename T>  \
	inline VECCLASS operator * (const MATCLASS & m, const VECCLASS & v) \
	{ \
		VECCLASS result; \
		for (int i = 0; i < SIZE; ++i) {\
			result[i] = dot(MATRIX_ROW ## SIZE(m, i), v); \
		} \
		return result; \
	} \
	template <typename T>  \
	inline VECCLASS operator * (const VECCLASS & v, const MATCLASS & m) \
	{ \
		VECCLASS result; \
		for (int i = 0; i < SIZE; ++i) \
			result[i] = dot(v, MATRIX_COL ## SIZE(m, i)); \
		return result; \
	}

MAT_VEC_FUNCS_TEMPLATE(mat2<T>, vec2<T>, 2)
MAT_VEC_FUNCS_TEMPLATE(mat3<T>, vec3<T>, 3)
MAT_VEC_FUNCS_TEMPLATE(mat4<T>, vec4<T>, 4)

// Returns the inverse of a 4x4 matrix. It is assumed that the matrix passed
// as argument describes a rigid-body transformation.
template <typename T> 
inline mat4<T> fast_inverse(const mat4<T>& m)
{
	const vec3<T> t = MATRIX_COL3(m, 3);
	const T tx = -dot(MATRIX_COL3(m, 0), t);
	const T ty = -dot(MATRIX_COL3(m, 1), t);
	const T tz = -dot(MATRIX_COL3(m, 2), t);

	return mat4<T>(
		m.elem[0][0], m.elem[1][0], m.elem[2][0], tx,
		m.elem[0][1], m.elem[1][1], m.elem[2][1], ty,
		m.elem[0][2], m.elem[1][2], m.elem[2][2], tz,
		T(0), T(0), T(0), T(1)
	);
}

// Transformations for points and vectors. Potentially faster than a full 
// matrix * vector multiplication

#define MAT_TRANFORMS_TEMPLATE(MATCLASS, VECCLASS, VECSIZE) \
	/* computes vec3<T>(m * vec4<T>(v, 0.0)) */ \
	template <typename T>  \
	inline VECCLASS transform_vector(const MATCLASS & m, const VECCLASS & v) \
	{ \
		VECCLASS result; \
		for (int i = 0; i < VECSIZE; ++i) \
			result[i] = dot(MATRIX_ROW ## VECSIZE(m, i), v); \
		return result;\
	} \
	/* computes vec3(m * vec4(v, 1.0)) */ \
	template <typename T>  \
	inline VECCLASS transform_point(const MATCLASS & m, const VECCLASS & v) \
	{ \
		/*return transform_vector(m, v) + MATRIX_ROW ## VECSIZE(m, VECSIZE); */\
		VECCLASS result; \
		for (int i = 0; i < VECSIZE; ++i) \
			result[i] = dot(MATRIX_ROW ## VECSIZE(m, i), v) + m.elem[i][VECSIZE]; \
		return result; \
	} \
	/* computes VECCLASS(transpose(m) * vec4<T>(v, 0.0)) */ \
	template <typename T>  \
	inline VECCLASS transform_vector_transpose(const MATCLASS & m, const VECCLASS& v) \
	{ \
		VECCLASS result; \
		for (int i = 0; i < VECSIZE; ++i) \
			result[i] = dot(MATRIX_COL ## VECSIZE(m, i), v); \
		return result; \
	} \
	/* computes VECCLASS(transpose(m) * vec4<T>(v, 1.0)) */ \
	template <typename T>  \
	inline VECCLASS transform_point_transpose(const MATCLASS & m, const VECCLASS& v) \
	{ \
		/*return transform_vector_transpose(m, v) + MATRIX_COL ## VECSIZE(m, VECSIZE); */\
		VECCLASS result; \
		for (int i = 0; i < VECSIZE; ++i) \
			result[i] = dot(MATRIX_COL ## VECSIZE(m, i), v) + m.elem[VECSIZE][i]; \
		return result; \
	}

MAT_TRANFORMS_TEMPLATE(mat4<T>, vec3<T>, 3)
MAT_TRANFORMS_TEMPLATE(mat3<T>, vec2<T>, 2)

#define MAT_OUTERPRODUCT_TEMPLATE(MATCLASS, VECCLASS, MATSIZE) \
	template <typename T> \
	inline MATCLASS outer_product(const VECCLASS & v1, const VECCLASS & v2) \
	{ \
		MATCLASS r; \
		for ( int j = 0; j < MATSIZE; ++j ) \
		for ( int k = 0; k < MATSIZE; ++k ) \
			r.elem[j][k] = v1[j] * v2[k]; \
		return r; \
	}

MAT_OUTERPRODUCT_TEMPLATE(mat4<T>, vec4<T>, 4)
MAT_OUTERPRODUCT_TEMPLATE(mat3<T>, vec3<T>, 3)
MAT_OUTERPRODUCT_TEMPLATE(mat2<T>, vec2<T>, 2)

template <typename T>
inline mat4<T> translation_matrix(const T x, const T y, const T z)
{
	mat4<T> r(T(1));
	r.elem[0][3] = x;
	r.elem[1][3] = y;
	r.elem[2][3] = z;
	return r;
}

template <typename T> 
inline mat4<T> translation_matrix(const vec3<T>& v)
{
	return translation_matrix(v.x, v.y, v.z);
}

template <typename T> 
inline mat4<T> scaling_matrix(const T x, const T y, const T z)
{
	mat4<T> r(T(0));
	r.elem[0][0] = x;
	r.elem[1][1] = y;
	r.elem[2][2] = z;
	r.elem[3][3] = T(1);
	return r;
}

template <typename T>
inline mat4<T> scaling_matrix(const vec3<T>& v)
{
	return scaling_matrix(v.x, v.y, v.z);
}

template <typename T>
inline mat4<T> rotation_matrix(const T angle, const vec3<T>& v)
{
	const T a = angle * T(M_PI/180) ;
	const vec3<T> u = normalize(v);
	
	const mat3<T> S(
		 T(0), -u[2],  u[1],
		 u[2],  T(0), -u[0],
		-u[1],  u[0],  T(0) 
	);
	
	const mat3<T> uut = outer_product(u, u);
	const mat3<T> R = uut + T(cos(a)) * (identity3<T>() - uut) + T(sin(a)) * S;
	
	return mat4<T>(R);
}


template <typename T> 
inline mat4<T> rotation_matrix(const T angle, const T x, const T y, const T z)
{
	return rotation_matrix(angle, vec3<T>(x, y, z));
}

// Constructs a shear-matrix that shears component i by factor with
// Respect to component j.
template <typename T> 
inline mat4<T> shear_matrix(const int i, const int j, const T factor)
{
	mat4<T> m = identity4<T>();
	m.elem[i][j] = factor;
	return m;
}

template <typename T> 
inline mat4<T> euler(const T head, const T pitch, const T roll)
{
	return rotation_matrix(roll, T(0), T(0), T(1)) * 
		rotation_matrix(pitch, T(1), T(0), T(0)) * 
		rotation_matrix(head, T(0), T(1), T(0));
}

template <typename T> 
inline mat4<T> frustum_matrix(const T l, const T r, const T b, const T t, const T n, const T f)
{
	return mat4<T>(
		(2 * n)/(r - l), T(0), (r + l)/(r - l), T(0),
		T(0), (2 * n)/(t - b), (t + b)/(t - b), T(0),
		T(0), T(0), -(f + n)/(f - n), -(2 * f * n)/(f - n),
		T(0), T(0), -T(1), T(0)
	);
}

template <typename T>
inline mat4<T> perspective_matrix(const T fovy, const T aspect, const T zNear, const T zFar)
{
	const T dz = zFar - zNear;
	const T rad = fovy / T(2) * T(M_PI/180);
	const T s = sin(rad);
	
	if ( ( dz == T(0) ) || ( s == T(0) ) || ( aspect == T(0) ) ) {
		return identity4<T>();
	}
	
	const T cot = cos(rad) / s;
	
	mat4<T> m = identity4<T>();
	m[0]  = cot / aspect;
	m[5]  = cot;
	m[10] = -(zFar + zNear) / dz;
	m[14] = T(-1);
	m[11] = -2 * zNear * zFar / dz;
	m[15] = T(0);
	
	return m;
}

template <typename T>
inline mat4<T> ortho_matrix(const T l, const T r, const T b, const T t, const T n, const T f)
{
	return mat4<T>(
		T(2)/(r - l), T(0), T(0), -(r + l)/(r - l),
		T(0), T(2)/(t - b), T(0), -(t + b)/(t - b),
		T(0), T(0), -T(2)/(f - n), -(f + n)/(f - n),
		T(0), T(0), T(0), T(1)
	);
}

template <typename T>
inline mat4<T> lookat_matrix(const vec3<T>& eye, const vec3<T>& center, const vec3<T>& up) {
	const vec3<T> forward = normalize(center - eye);
	const vec3<T> side = normalize(cross(forward, up));
	
	const vec3<T> up2 = cross(side, forward);
	
	mat4<T> m = identity4<T>();
	
	m.elem[0][0] = side[0];
	m.elem[0][1] = side[1];
	m.elem[0][2] = side[2];
	
	m.elem[1][0] = up2[0];
	m.elem[1][1] = up2[1];
	m.elem[1][2] = up2[2];
	
	m.elem[2][0] = -forward[0];
	m.elem[2][1] = -forward[1];
	m.elem[2][2] = -forward[2];
	
	return m * translation_matrix(-eye);
}

template <typename T> 
inline mat4<T> picking_matrix(const T x, const T y, const T dx, const T dy, int viewport[4]) {
	if (dx <= 0 || dy <= 0) { 
		return identity4<T>();
	}

	mat4<T> r = translation_matrix((viewport[2] - 2 * (x - viewport[0])) / dx,
		(viewport[3] - 2 * (y - viewport[1])) / dy,	0);
	r *= scaling_matrix(viewport[2] / dx, viewport[2] / dy, 1);
	return r;
}

// Constructs a shadow matrix. q is the light source and p is the plane.
template <typename T> inline mat4<T> shadow_matrix(const vec4<T>& q, const vec4<T>& p) {
	mat4<T> m;
	
	m.elem[0][0] =  p.y * q[1] + p.z * q[2] + p.w * q[3];
	m.elem[0][1] = -p.y * q[0];
	m.elem[0][2] = -p.z * q[0];
	m.elem[0][3] = -p.w * q[0];
	
	m.elem[1][0] = -p.x * q[1];
	m.elem[1][1] =  p.x * q[0] + p.z * q[2] + p.w * q[3];
	m.elem[1][2] = -p.z * q[1];
	m.elem[1][3] = -p.w * q[1];
	

	m.elem[2][0] = -p.x * q[2];
	m.elem[2][1] = -p.y * q[2];
	m.elem[2][2] =  p.x * q[0] + p.y * q[1] + p.w * q[3];
	m.elem[2][3] = -p.w * q[2];

	m.elem[3][1] = -p.x * q[3];
	m.elem[3][2] = -p.y * q[3];
	m.elem[3][3] = -p.z * q[3];
	m.elem[3][0] =  p.x * q[0] + p.y * q[1] + p.z * q[2];

	return m;
}

// Quaternion class
template <typename T> 
struct quat {
	vec3<T>	v;
	T w;

	quat() {}
	quat(const vec3<T>& iv, const T iw) : v(iv), w(iw) {}
	quat(const T vx, const T vy, const T vz, const T iw) : v(vx, vy, vz), w(iw)	{}
	quat(const vec4<T>& i) : v(i.x, i.y, i.z), w(i.w) {}

	operator const T* () const { return &(v[0]); }
	operator T* () { return &(v[0]); }	

	quat& operator += (const quat& q) { v += q.v; w += q.w; return *this; }
	quat& operator -= (const quat& q) { v -= q.v; w -= q.w; return *this; }

	quat& operator *= (const T& s) { v *= s; w *= s; return *this; }
	quat& operator /= (const T& s) { v /= s; w /= s; return *this; }

	quat& operator *= (const quat& r)
	{
		//q1 x q2 = [s1,v1] x [s2,v2] = [(s1*s2 - v1*v2),(s1*v2 + s2*v1 + v1xv2)].
		quat q;
		q.v = cross(v, r.v) + r.w * v + w * r.v;
		q.w = w * r.w - dot(v, r.v);
		return *this = q;
	}

	quat& operator /= (const quat& q) { return (*this) *= inverse(q); }
};

// Quaternion functions

template <typename T> 
inline quat<T> identityq()
{
	return quat<T>(T(0), T(0), T(0), T(1));
}

template <typename T> 
inline quat<T> conjugate(const quat<T>& q)
{
	return quat<T>(-q.v, q.w);
}

template <typename T> 
inline quat<T> inverse(const quat<T>& q)
{
	const T l = dot(q, q);
	if ( l > T(0) ) return conjugate(q) * inv(l);
	else return identityq<T>();
}

// quaternion utility functions

// the input quaternion is assumed to be normalized
template <typename T> 
inline mat3<T> quat_to_mat3(const quat<T>& q)
{
	// const quat<T> q = normalize(qq);

	const T xx = q[0] * q[0];
	const T xy = q[0] * q[1];
	const T xz = q[0] * q[2];
	const T xw = q[0] * q[3];
	
	const T yy = q[1] * q[1];
	const T yz = q[1] * q[2];
	const T yw = q[1] * q[3];
	
	const T zz = q[2] * q[2];
	const T zw = q[2] * q[3];
	
	return mat3<T>(
		1 - 2*(yy + zz), 2*(xy - zw), 2*(xz + yw),
		2*(xy + zw), 1 - 2*(xx + zz), 2*(yz - xw),
		2*(xz - yw), 2*(yz + xw), 1 - 2*(xx + yy)
	);
}

// the input quat<T>ernion is assumed to be normalized
template <typename T> 
inline mat4<T> quat_to_mat4(const quat<T>& q)
{
	// const quat<T> q = normalize(qq);

	return mat4<T>(quat_to_mat3(q));
}

template <typename T> 
inline quat<T> mat_to_quat(const mat4<T>& m)
{
	const T t = m.elem[0][0] + m.elem[1][1] + m.elem[2][2] + T(1);
	quat<T> q;

	if ( t > 0 ) {
		const T s = T(0.5) / sqrt(t);
		q[3] = T(0.25) * inv(s);
		q[0] = (m.elem[2][1] - m.elem[1][2]) * s;
		q[1] = (m.elem[0][2] - m.elem[2][0]) * s;
		q[2] = (m.elem[1][0] - m.elem[0][1]) * s;
	} else {
		if ( m.elem[0][0] > m.elem[1][1] && m.elem[0][0] > m.elem[2][2] ) {
			const T s = T(2) * sqrt( T(1) + m.elem[0][0] - m.elem[1][1] - m.elem[2][2]);
			const T invs = inv(s);
			q[0] = T(0.25) * s;
			q[1] = (m.elem[0][1] + m.elem[1][0] ) * invs;
			q[2] = (m.elem[0][2] + m.elem[2][0] ) * invs;
			q[3] = (m.elem[1][2] - m.elem[2][1] ) * invs;
		} else if (m.elem[1][1] > m.elem[2][2]) {
			const T s = T(2) * sqrt( T(1) + m.elem[1][1] - m.elem[0][0] - m.elem[2][2]);
			const T invs = inv(s);
			q[0] = (m.elem[0][1] + m.elem[1][0] ) * invs;
			q[1] = T(0.25) * s;
			q[2] = (m.elem[1][2] + m.elem[2][1] ) * invs;
			q[3] = (m.elem[0][2] - m.elem[2][0] ) * invs;
		} else {
			const T s = T(2) * sqrt( T(1) + m.elem[2][2] - m.elem[0][0] - m.elem[1][1] );
			const T invs = inv(s);
			q[0] = (m.elem[0][2] + m.elem[2][0] ) * invs;
			q[1] = (m.elem[1][2] + m.elem[2][1] ) * invs;
			q[2] = T(0.25) * s;
			q[3] = (m.elem[0][1] - m.elem[1][0] ) * invs;
		}
	}
	
	return q;
}

template <typename T> 
inline quat<T> mat_to_quat(const mat3<T>& m)
{
	return mat_to_quat(mat4<T>(m));
}

// the angle is in radians
template <typename T> 
inline quat<T> quat_from_axis_angle(const vec3<T>& axis, const T a)
{
	quat<T> r;
	const T inv2 = inv(T(2));
	r.v = sin(a * inv2) * normalize(axis);
	r.w = cos(a * inv2);
	
	return r;
}

// the angle is in radians
template <typename T> 
inline quat<T> quat_from_axis_angle(const T x, const T y, const T z, const T angle)
{
	return quat_from_axis_angle<T>(vec3<T>(x, y, z), angle);
}

// the angle is stored in radians
template <typename T> 
inline void quat_to_axis_angle(const quat<T>& qq, vec3<T>* axis, T *angle)
{
	quat<T> q = normalize(qq);
	
	*angle = 2 * acos(q.w);
	
	const T s = sin((*angle) * inv(T(2)));
	if ( s != T(0) )
		*axis = q.v * inv(s);
	else 
		* axis = vec3<T>(T(0), T(0), T(0));
}

// Spherical linear interpolation
template <typename T> 
inline quat<T> slerp(const quat<T>& qq1, const quat<T>& qq2, const T t) 
{
	// slerp(q1,q2) = sin((1-t)*a)/sin(a) * q1  +  sin(t*a)/sin(a) * q2
	const quat<T> q1 = normalize(qq1);
	const quat<T> q2 = normalize(qq2);
	
	const T a = acos(dot(q1, q2));
	const T s = sin(a);
	
	#define EPS T(1e-5)

	if ( !(-EPS <= s && s <= EPS) ) {
		return sin((T(1)-t)*a)/s * q1  +  sin(t*a)/s * q2;
	} else {
		// if the angle is to small use a linear interpolation
		return lerp(q1, q2, t);
	}

	#undef EPS
}

// Sperical quadtratic interpolation using a smooth cubic spline
// The parameters a and b are the control points.
template <typename T> 
inline quat<T> squad(
	const quat<T>& q0, 
	const quat<T>& a, 
	const quat<T>& b, 
	const quat<T>& q1, 
	const T t) 
{
	return slerp(slerp(q0, q1, t),slerp(a, b, t), 2 * t * (1 - t));
}

#undef MOP_M_CLASS_TEMPLATE
#undef MOP_M_TYPE_TEMPLATE
#undef MOP_COMP_TEMPLATE
#undef MOP_G_UMINUS_TEMPLATE
#undef COMMON_OPERATORS
#undef VECTOR_COMMON
#undef FOP_G_SOURCE_TEMPLATE
#undef FOP_G_CLASS_TEMPLATE
#undef FOP_G_TYPE_TEMPLATE
#undef VEC_QUAT_FUNC_TEMPLATE
#undef VEC_FUNC_TEMPLATE
#undef MATRIX_COL4
#undef MATRIX_ROW4
#undef MATRIX_COL3
#undef MATRIX_ROW3
#undef MATRIX_COL2
#undef MATRIX_ROW2
#undef MOP_M_MATRIX_MULTIPLY
#undef MATRIX_CONSTRUCTOR_FROM_T
#undef MATRIX_CONSTRUCTOR_FROM_LOWER
#undef MATRIX_COMMON
#undef MATRIX_CONSTRUCTOR_FROM_HIGHER
#undef MAT_FUNC_TEMPLATE 
#undef MAT_FUNC_MINOR_TEMPLATE
#undef MAT_ADJOINT_TEMPLATE
#undef MAT_INVERSE_TEMPLATE
#undef MAT_VEC_FUNCS_TEMPLATE
#undef MAT_TRANFORMS_TEMPLATE
#undef MAT_OUTERPRODUCT_TEMPLATE
#undef FREE_MODIFYING_OPERATORS
#undef FREE_OPERATORS

} // end namespace vmath

#endif


