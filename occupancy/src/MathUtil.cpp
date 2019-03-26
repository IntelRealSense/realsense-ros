#include "MathUtil.h"
#include <iterator>
#include "SP_CrossPlatformsFunctions.h"
#include <cstring>
#include <cassert>
#include <memory>
#include <algorithm>

using namespace ScenePerception;
template<>
bool iVector4<float, cl_float4, true>::operator==(const
        iVector4<float, cl_float4, true> &rhs) const
{
    return (abs(x - rhs.x) < PRECISION) && (abs(y - rhs.y) < PRECISION)
           && (abs(z - rhs.z) < PRECISION) && (abs(w - rhs.w) < PRECISION);
}

template<>
bool iVector4<float, cl_float4, true>::operator==(float a) const
{
    return (abs(x - a) < PRECISION) && (abs(y - a) < PRECISION)
           && (abs(z - a) < PRECISION) && (abs(w - a) < PRECISION);
}

template<>
bool iVector4<float, cl_float4, false>::operator==(const
        iVector4<float, cl_float4, false> &rhs) const
{
    return (abs(x - rhs.x) < PRECISION) && (abs(y - rhs.y) < PRECISION)
           && (abs(z - rhs.z) < PRECISION);
}

template<>
bool iVector4<float, cl_float4, false>::operator==(float a) const
{
    return (abs(x - a) < PRECISION) && (abs(y - a) < PRECISION)
           && (abs(z - a) < PRECISION);
}

template<>
bool Vector3<float, cl_float4>::operator==(const Vector3<float, cl_float4> &rhs)
const
{
    return (abs(x - rhs.x) < PRECISION) && (abs(y - rhs.y) < PRECISION)
           && (abs(z - rhs.z) < PRECISION);
}

template<>
bool Vector3<float, cl_float4>::operator==(float a) const
{
    return (abs(x - a) < PRECISION) && (abs(y - a) < PRECISION)
           && (abs(z - a) < PRECISION);
}


template<>
bool iVector2<float, cl_float2>::operator==(const iVector2<float, cl_float2>
        &rhs) const
{
    return (abs(x - rhs.x) < PRECISION) && (abs(y - rhs.y) < PRECISION);
}

template<>
bool iVector2<float, cl_float2>::operator==(float a) const
{
    return (abs(x - a) < PRECISION) && (abs(y - a) < PRECISION);
}

Matrix4f Matrix4f::Identity(1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 1, 0,
                            0, 0, 0, 1);

Matrix4f Matrix4f::Zero(0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0,
                        0, 0, 0, 0);

float4 Matrix4f::Transform(const Matrix4f& mat, const float4& vec)
{
    return float4(
               vec.x * mat.m_data[0] + vec.y * mat.m_data[4] + vec.z * mat.m_data[8] + vec.w *
               mat.m_data[12],
               vec.x * mat.m_data[1] + vec.y * mat.m_data[5] + vec.z * mat.m_data[9] + vec.w *
               mat.m_data[13],
               vec.x * mat.m_data[2] + vec.y * mat.m_data[6] + vec.z * mat.m_data[10] + vec.w *
               mat.m_data[14],
               vec.x * mat.m_data[3] + vec.y * mat.m_data[7] + vec.z * mat.m_data[11] + vec.w *
               mat.m_data[15]);
}

float3 Matrix4f::Transform(const Matrix4f& M, const float3& vec)
{
    float4 v4(vec.x, vec.y, vec.z, 1);
    float4 vresult = Matrix4f::Transform(M, v4);
    return float3(vresult.x, vresult.y, vresult.z);
}

Matrix4f ScenePerception::GetCameraMatrix(const float4 &kParam)
{
    return Matrix4f(kParam.x, 0.0f, kParam.z, 0.0f,
                    0.0f, kParam.y, kParam.w, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f);
}

Matrix4f ScenePerception::GetInverseCameraMatrix(const float4 &kParam)
{
    // Assumes skew = 0
    return Matrix4f(1.0f / kParam.x, 0.0f, -kParam.z / kParam.x, 0.0f,
                    0.0f, 1.0f / kParam.y, -kParam.w / kParam.y, 0.0f,
                    0.0f, 0.0f, 1.0f, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f);
}

Matrix4f::Matrix4f(const float val)
{
    for (int i = 0; i < 16; i++)
    {
        m_data[i] = val;
    }
}

Matrix4f::Matrix4f(float m00, float m01, float m02, float m03, 
	float m10, float m11, float m12, float m13, 
	float m20, float m21, float m22, float m23,
	float m30, float m31, float m32, float m33)
{
    m_data[0] = m00;
    m_data[1] = m01;
    m_data[2] = m02;
    m_data[3] = m03;
    m_data[4] = m10;
    m_data[5] = m11;
    m_data[6] = m12;
    m_data[7] = m13;
    m_data[8] = m20;
    m_data[9] = m21;
    m_data[10] = m22;
    m_data[11] = m23;
    m_data[12] = m30;
    m_data[13] = m31;
    m_data[14] = m32;
    m_data[15] = m33;
}

Matrix4f::Matrix4f()
{
    memset(m_data, 0, sizeof(float) * 16);
}

Matrix4f::Matrix4f(const std::vector<float>& vals)
{
    for (int i = 0; i < 16; i++)
    {
        m_data[i] = vals[i];
    }
}

Matrix4f::Matrix4f(const float(&vals)[16])
{
    SP_copy(vals, vals + 16, m_data, 16);
}

Matrix4f::Matrix4f(const float(&vals)[12])
{
    SP_copy(vals, vals + 12, m_data, 12);
    m_data[12] = m_data[13] = m_data[14] = 0;
    m_data[15] = 1.0f;
}

Matrix4f Matrix4f::Transpose() const
{
    return Matrix4f(m_data[0], m_data[4], m_data[8], m_data[12],
                    m_data[1], m_data[5], m_data[9], m_data[13],
                    m_data[2], m_data[6], m_data[10], m_data[14],
                    m_data[3], m_data[7], m_data[11], m_data[15]);
}

Matrix4f Matrix4f::Inverse() const
{
    float outData[16];

    const float v1[] = { m_data[4], m_data[5], m_data[6], m_data[7] };
    const float v2[] = { m_data[8], m_data[9], m_data[10], m_data[11] };
    const float v3[] = { m_data[12], m_data[13], m_data[14], m_data[15] };

    const float v00 = v1[2] * v2[3] * v3[1] - v1[3] * v2[2] * v3[1] + v1[3] * v2[1]
                      * v3[2] - v1[1] * v2[3] * v3[2] - v1[2] * v2[1] * v3[3] + v1[1] * v2[2] * v3[3];
    const float v10 = v1[3] * v2[2] * v3[0] - v1[2] * v2[3] * v3[0] - v1[3] * v2[0]
                      * v3[2] + v1[0] * v2[3] * v3[2] + v1[2] * v2[0] * v3[3] - v1[0] * v2[2] * v3[3];
    const float v20 = v1[1] * v2[3] * v3[0] - v1[3] * v2[1] * v3[0] + v1[3] * v2[0]
                      * v3[1] - v1[0] * v2[3] * v3[1] - v1[1] * v2[0] * v3[3] + v1[0] * v2[1] * v3[3];
    const float v30 = v1[2] * v2[1] * v3[0] - v1[1] * v2[2] * v3[0] - v1[2] * v2[0]
                      * v3[1] + v1[0] * v2[2] * v3[1] + v1[1] * v2[0] * v3[2] - v1[0] * v2[1] * v3[2];

    const float v0[] = { m_data[0], m_data[1], m_data[2], m_data[3] };

    const float vdet = v0[3] * v1[2] * v2[1] * v3[0] - v0[3] * v1[1] * v2[2] * v3[0]
                       - v0[2] * v1[3] * v2[1] * v3[0] + v0[1] * v1[3] * v2[2] * v3[0] +
                       v0[2] * v1[1] * v2[3] * v3[0] - v0[1] * v1[2] * v2[3] * v3[0] - v0[3] * v1[2] *
                       v2[0] * v3[1] + v0[2] * v1[3] * v2[0] * v3[1] +
                       v0[3] * v1[0] * v2[2] * v3[1] - v0[2] * v1[0] * v2[3] * v3[1] - v0[0] * v1[3] *
                       v2[2] * v3[1] + v0[0] * v1[2] * v2[3] * v3[1] +
                       v0[3] * v1[1] * v2[0] * v3[2] - v0[1] * v1[3] * v2[0] * v3[2] - v0[3] * v1[0] *
                       v2[1] * v3[2] + v0[0] * v1[3] * v2[1] * v3[2] +
                       v0[1] * v1[0] * v2[3] * v3[2] - v0[2] * v1[1] * v2[0] * v3[3] - v0[0] * v1[1] *
                       v2[3] * v3[2] + v0[1] * v1[2] * v2[0] * v3[3] +
                       v0[2] * v1[0] * v2[1] * v3[3] - v0[0] * v1[2] * v2[1] * v3[3] - v0[1] * v1[0] *
                       v2[2] * v3[3] + v0[0] * v1[1] * v2[2] * v3[3];

    const float v01 = v0[3] * v2[2] * v3[1] - v0[2] * v2[3] * v3[1] - v0[3] * v2[1]
                      * v3[2] + v0[1] * v2[3] * v3[2] + v0[2] * v2[1] * v3[3] - v0[1] * v2[2] * v3[3];
    const float v02 = v0[2] * v1[3] * v3[1] - v0[3] * v1[2] * v3[1] + v0[3] * v1[1]
                      * v3[2] - v0[1] * v1[3] * v3[2] - v0[2] * v1[1] * v3[3] + v0[1] * v1[2] * v3[3];
    const float v03 = v0[3] * v1[2] * v2[1] - v0[2] * v1[3] * v2[1] - v0[3] * v1[1]
                      * v2[2] + v0[1] * v1[3] * v2[2] + v0[2] * v1[1] * v2[3] - v0[1] * v1[2] * v2[3];

    float invdet = 0;
    if (fabs(vdet) > 0.0000001f)
    {
        invdet = 1.0f / vdet;
    }

    outData[0] = v00 * invdet;
    outData[1] = v01 * invdet;
    outData[2] = v02 * invdet;
    outData[3] = v03 * invdet;

    const float v11 = v0[2] * v2[3] * v3[0] - v0[3] * v2[2] * v3[0] + v0[3] * v2[0]
                      * v3[2] - v0[0] * v2[3] * v3[2] - v0[2] * v2[0] * v3[3] + v0[0] * v2[2] * v3[3];
    const float v12 = v0[3] * v1[2] * v3[0] - v0[2] * v1[3] * v3[0] - v0[3] * v1[0]
                      * v3[2] + v0[0] * v1[3] * v3[2] + v0[2] * v1[0] * v3[3] - v0[0] * v1[2] * v3[3];
    const float v13 = v0[2] * v1[3] * v2[0] - v0[3] * v1[2] * v2[0] + v0[3] * v1[0]
                      * v2[2] - v0[0] * v1[3] * v2[2] - v0[2] * v1[0] * v2[3] + v0[0] * v1[2] * v2[3];

    outData[4] = v10 * invdet;
    outData[5] = v11 * invdet;
    outData[6] = v12 * invdet;
    outData[7] = v13 * invdet;

    const float v21 = v0[3] * v2[1] * v3[0] - v0[1] * v2[3] * v3[0] - v0[3] * v2[0]
                      * v3[1] + v0[0] * v2[3] * v3[1] + v0[1] * v2[0] * v3[3] - v0[0] * v2[1] * v3[3];
    const float v22 = v0[1] * v1[3] * v3[0] - v0[3] * v1[1] * v3[0] + v0[3] * v1[0]
                      * v3[1] - v0[0] * v1[3] * v3[1] - v0[1] * v1[0] * v3[3] + v0[0] * v1[1] * v3[3];
    const float v23 = v0[3] * v1[1] * v2[0] - v0[1] * v1[3] * v2[0] - v0[3] * v1[0]
                      * v2[1] + v0[0] * v1[3] * v2[1] + v0[1] * v1[0] * v2[3] - v0[0] * v1[1] * v2[3];

    outData[8] = v20 * invdet;
    outData[9] = v21 * invdet;
    outData[10] = v22 * invdet;
    outData[11] = v23 * invdet;

    const float v31 = v0[1] * v2[2] * v3[0] - v0[2] * v2[1] * v3[0] + v0[2] * v2[0]
                      * v3[1] - v0[0] * v2[2] * v3[1] - v0[1] * v2[0] * v3[2] + v0[0] * v2[1] * v3[2];
    const float v32 = v0[2] * v1[1] * v3[0] - v0[1] * v1[2] * v3[0] - v0[2] * v1[0]
                      * v3[1] + v0[0] * v1[2] * v3[1] + v0[1] * v1[0] * v3[2] - v0[0] * v1[1] * v3[2];
    const float v33 = v0[1] * v1[2] * v2[0] - v0[2] * v1[1] * v2[0] + v0[2] * v1[0]
                      * v2[1] - v0[0] * v1[2] * v2[1] - v0[1] * v1[0] * v2[2] + v0[0] * v1[1] * v2[2];

    outData[12] = v30 * invdet;
    outData[13] = v31 * invdet;
    outData[14] = v32 * invdet;
    outData[15] = v33 * invdet;

    return Matrix4f(outData);
}

void Matrix4f::Set(const std::vector<float>& M)
{
    if (M.size() == 16)
    {
        SP_copy(&M[0], &M[0] + 16, m_data, 16);
    }
}

void Matrix4f::Get(std::vector<float>& M) const
{
    M = std::vector<float>(m_data, m_data + 16);
}

float Matrix4f::At(int i, int j) const
{
    return m_data[j + i * 4];
}

void Matrix4f::Set(int i, int j, float newValue)
{
    m_data[j + i * 4] = newValue;
}

Matrix4f Matrix4f::MultiplyWith(const Matrix4f& M) const
{
    Matrix4f result;

    const float a00 = m_data[0],    a01 = m_data[1],    a02 = m_data[2],
                a03 = m_data[3],
                a10 = m_data[4],    a11 = m_data[5],    a12 = m_data[6],    a13 = m_data[7],
                a20 = m_data[8],    a21 = m_data[9],    a22 = m_data[10],   a23 = m_data[11],
                a30 = m_data[12],   a31 = m_data[13],   a32 = m_data[14],   a33 = m_data[15],
                b00 = M.m_data[0],  b01 = M.m_data[1],  b02 = M.m_data[2],  b03 = M.m_data[3],
                b10 = M.m_data[4],  b11 = M.m_data[5],  b12 = M.m_data[6],  b13 = M.m_data[7],
                b20 = M.m_data[8],  b21 = M.m_data[9],  b22 = M.m_data[10], b23 = M.m_data[11],
                b30 = M.m_data[12], b31 = M.m_data[13], b32 = M.m_data[14], b33 = M.m_data[15];

    float *r = result.m_data;
    *r++ = (a00 * b00) + (a01 * b10) + (a02 * b20) + (a03 * b30);
    *r++ = (a00 * b01) + (a01 * b11) + (a02 * b21) + (a03 * b31);
    *r++ = (a00 * b02) + (a01 * b12) + (a02 * b22) + (a03 * b32);
    *r++ = (a00 * b03) + (a01 * b13) + (a02 * b23) + (a03 * b33);

    *r++ = (a10 * b00) + (a11 * b10) + (a12 * b20) + (a13 * b30);
    *r++ = (a10 * b01) + (a11 * b11) + (a12 * b21) + (a13 * b31);
    *r++ = (a10 * b02) + (a11 * b12) + (a12 * b22) + (a13 * b32);
    *r++ = (a10 * b03) + (a11 * b13) + (a12 * b23) + (a13 * b33);

    *r++ = (a20 * b00) + (a21 * b10) + (a22 * b20) + (a23 * b30);
    *r++ = (a20 * b01) + (a21 * b11) + (a22 * b21) + (a23 * b31);
    *r++ = (a20 * b02) + (a21 * b12) + (a22 * b22) + (a23 * b32);
    *r++ = (a20 * b03) + (a21 * b13) + (a22 * b23) + (a23 * b33);

    *r++ = (a30 * b00) + (a31 * b10) + (a32 * b20) + (a33 * b30);
    *r++ = (a30 * b01) + (a31 * b11) + (a32 * b21) + (a33 * b31);
    *r++ = (a30 * b02) + (a31 * b12) + (a32 * b22) + (a33 * b32);
    *r   = (a30 * b03) + (a31 * b13) + (a32 * b23) + (a33 * b33);

    return result;
}

Matrix4f Matrix4f::operator-(const Matrix4f& M) const
{
    Matrix4f Result;
    for (int i = 0; i < 16; i++)
    {
        Result.m_data[i] = m_data[i] - M.m_data[i];
    }
    return Result;
}

Matrix4f Matrix4f::operator+(const Matrix4f& M) const
{
    Matrix4f Result;
    for (int i = 0; i < 16; i++)
    {
        Result.m_data[i] = m_data[i] + M.m_data[i];
    }
    return Result;
}

Matrix4f Matrix4f::operator+(const float val) const
{
    Matrix4f Result;
    for (int i = 0; i < 16; i++)
    {
        Result.m_data[i] = m_data[i] + val;
    }
    return Result;
}

Matrix4f Matrix4f::operator-(const float val) const
{
    Matrix4f Result;
    for (int i = 0; i < 16; i++)
    {
        Result.m_data[i] = m_data[i] - val;
    }
    return Result;
}

Matrix4f Matrix4f::operator*(const float val) const
{
    return Matrix4f(m_data[0]  * val,  m_data[1]  * val, m_data[2]  * val,
                    m_data[3]  * val,
                    m_data[4]  * val,  m_data[5]  * val, m_data[6]  * val, m_data[7]  * val,
                    m_data[8]  * val,  m_data[9]  * val, m_data[10] * val, m_data[11] * val,
                    m_data[12] * val,  m_data[13] * val, m_data[14] * val, m_data[15] * val);
}

Matrix4f Matrix4f::operator*(const Matrix4f& M) const
{
    return MultiplyWith(M);
}

ScenePerception::float4 Matrix4f::operator*(const float4& v) const
{
    return float4((*(m_data)* v.x) + (*(m_data + 1)  * v.y) + (*
                  (m_data + 2)  * v.z) + (*(m_data + 3)  * v.w),
                  (*(m_data + 4) * v.x) + (*(m_data + 5)  * v.y) + (*(m_data + 6)  * v.z) + (*
                          (m_data + 7)  * v.w),
                  (*(m_data + 8) * v.x) + (*(m_data + 9)  * v.y) + (*(m_data + 10) * v.z) + (*
                          (m_data + 11) * v.w),
                  (*(m_data + 12) * v.x) + (*(m_data + 13) * v.y) + (*(m_data + 14) * v.z) + (*
                          (m_data + 15) * v.w));
}

Matrix4f Matrix4f::operator/(const float val) const
{
    Matrix4f Result;
    for (int i = 0; i < 16; i++)
    {
        Result.m_data[i] = m_data[i] / val;
    }
    return Result;
}

Matrix4f& Matrix4f::operator=(const Matrix4f& M)
{
    if (this != &M)
    {
        SP_copy(M.m_data, M.m_data + 16, m_data, 16);
    }
    return *this;
}

Matrix4f Matrix4f::MakeUnitVectorRotation(const float4& v1, const float4& v2) // needed?
{
	const float4 v = cross(v1, v2);
	const float s = v.length();
	if (s == 0.0f)
	{
		return Matrix4f::Identity;
	}

	Matrix4f A(
		0.0f, -v.z, v.y, 0.0f,
		v.z, 0.0f, -v.x, 0.0f,
		-v.y, v.x, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f);

	float c = v1.dot(v2);
	c = (1.0f - c) / (s*s);
	return Matrix4f::Identity + A + A * A * c;
}

PoseMatrix4f PoseMatrix4f::operator*(const PoseMatrix4f& poseMat2) const
{
    const float
    a00 = m_data[0], a01 = m_data[1], a02 = m_data[2], a03 = m_data[3],
    a10 = m_data[4], a11 = m_data[5], a12 = m_data[6], a13 = m_data[7],
    a20 = m_data[8], a21 = m_data[9], a22 = m_data[10], a23 = m_data[11],

    b00 = poseMat2.m_data[0], b01 = poseMat2.m_data[1], b02 = poseMat2.m_data[2],
    b03 = poseMat2.m_data[3],
    b10 = poseMat2.m_data[4], b11 = poseMat2.m_data[5], b12 = poseMat2.m_data[6],
    b13 = poseMat2.m_data[7],
    b20 = poseMat2.m_data[8], b21 = poseMat2.m_data[9], b22 = poseMat2.m_data[10],
    b23 = poseMat2.m_data[11];


    return PoseMatrix4f(
               (a00 * b00) + (a01 * b10) + (a02 * b20),
               (a00 * b01) + (a01 * b11) + (a02 * b21),
               (a00 * b02) + (a01 * b12) + (a02 * b22),
               (a00 * b03) + (a01 * b13) + (a02 * b23) + a03,

               (a10 * b00) + (a11 * b10) + (a12 * b20),
               (a10 * b01) + (a11 * b11) + (a12 * b21),
               (a10 * b02) + (a11 * b12) + (a12 * b22),
               (a10 * b03) + (a11 * b13) + (a12 * b23) + a13,

               (a20 * b00) + (a21 * b10) + (a22 * b20),
               (a20 * b01) + (a21 * b11) + (a22 * b21),
               (a20 * b02) + (a21 * b12) + (a22 * b22),
               (a20 * b03) + (a21 * b13) + (a22 * b23) + a23);
}

ScenePerception::float4 ScenePerception::PoseMatrix4f::operator*(const float4& v) const
{
    return float4(
               (*(m_data    ) * v.x) + (*(m_data + 1) * v.y) + (*(m_data + 2 ) * v.z) + (*(m_data + 3 ) * v.w),
               (*(m_data + 4) * v.x) + (*(m_data + 5) * v.y) + (*(m_data + 6 ) * v.z) + (*(m_data + 7 ) * v.w),
               (*(m_data + 8) * v.x) + (*(m_data + 9) * v.y) + (*(m_data + 10) * v.z) + (*(m_data + 11) * v.w),
               v.w);
}

float3 ScenePerception::PoseMatrix4f::operator*(const float3& v) const
{
    return float3(
               (*(m_data)     * v.x) + (*(m_data + 1)  * v.y) + (*(m_data + 2 ) * v.z) + (*(m_data + 3 )),
               (*(m_data + 4) * v.x) + (*(m_data + 5)  * v.y) + (*(m_data + 6 ) * v.z) + (*(m_data + 7 )),
               (*(m_data + 8) * v.x) + (*(m_data + 9)  * v.y) + (*(m_data + 10) * v.z) + (*(m_data + 11)));
}

PoseMatrix4f::PoseMatrix4f() : Matrix4f(
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f)
{

}
PoseMatrix4f& PoseMatrix4f::operator=(const PoseMatrix4f& poseMat)
{
    if (this != &poseMat && poseMat.m_data != nullptr)
    {
        SP_memcpy(m_data, sizeof(m_data), poseMat.m_data, sizeof(poseMat.m_data));
    }
    return *this;
}

PoseMatrix4f::PoseMatrix4f(const PoseMatrix4f &poseMat) : Matrix4f(
        poseMat.m_data[0], poseMat.m_data[1], poseMat.m_data[2], poseMat.m_data[3],
        poseMat.m_data[4], poseMat.m_data[5], poseMat.m_data[6], poseMat.m_data[7],
        poseMat.m_data[8], poseMat.m_data[9], poseMat.m_data[10], poseMat.m_data[11],
        0.0f, 0.0f, 0.0f, 1.0f)
{

}

PoseMatrix4f::PoseMatrix4f(const std::vector<float>& vals) : Matrix4f(vals[0],
            vals[1], vals[2], vals[3],
            vals[4], vals[5], vals[6], vals[7],
            vals[8], vals[9], vals[10], vals[11],
            0.0f, 0.0f, 0.0f, 1.0f)
{
    assert(vals.size() >= 12);
}

PoseMatrix4f::PoseMatrix4f(const float3& rotationMatRow1,
                           const float3& rotationMatRow2, const float3& rotationMatRow3,
                           const float3& translationVector) :
    Matrix4f(rotationMatRow1.x, rotationMatRow1.y, rotationMatRow1.z,
             translationVector.x,
             rotationMatRow2.x, rotationMatRow2.y, rotationMatRow2.z, translationVector.y,
             rotationMatRow3.x, rotationMatRow3.y, rotationMatRow3.z, translationVector.z,
             0.0f, 0.0f, 0.0f, 1.0f)
{

}

PoseMatrix4f::PoseMatrix4f(const float(&rotationMat)[9],
                           const float3 & translationVector) :
    Matrix4f(rotationMat[0], rotationMat[1], rotationMat[2], translationVector.x,
             rotationMat[3], rotationMat[4], rotationMat[5], translationVector.y,
             rotationMat[6], rotationMat[7], rotationMat[8], translationVector.z,
             0.0f, 0.0f, 0.0f, 1.0f)
{

}

PoseMatrix4f::PoseMatrix4f(const float(&rotationMat)[3][3], const float(&translationVector)[3]) :
    Matrix4f(rotationMat[0][0], rotationMat[0][1], rotationMat[0][2], translationVector[0],
             rotationMat[1][0], rotationMat[1][1], rotationMat[1][2], translationVector[1],
             rotationMat[2][0], rotationMat[2][1], rotationMat[2][2], translationVector[2],
             0.0f, 0.0f, 0.0f, 1.0f)
{

}

PoseMatrix4f::PoseMatrix4f(float m00, float m01, float m02, float m03,
                           float m10, float m11, float m12, float m13,
                           float m20, float m21, float m22, float m23) :
    Matrix4f(m00, m01, m02, m03,
             m10, m11, m12, m13,
             m20, m21, m22, m23,
             0.0f, 0.0f, 0.0f, 1.0f)
{

}

PoseMatrix4f::PoseMatrix4f(const float(&vals)[12])
{
    SP_copy(vals, vals + 12, m_data, 12);
    m_data[12] = 0;
    m_data[13] = 0;
    m_data[14] = 0;
    m_data[15] = 1;
}

// needed?
PoseMatrix4f::PoseMatrix4f(const float(&pose)[6])
{
    convert6DOFToMatrix(pose, 1e-6);
}

PoseMatrix4f::PoseMatrix4f(const float(&pose)[6], double sqrtPrecision)
{
    convert6DOFToMatrix(pose, sqrtPrecision);
}

void PoseMatrix4f::convert6DOFToMatrix(const float(&pose)[6],
                                       double sqrtPrecision)
{
    m_data[3] = pose[0];
    m_data[7] = pose[1];
    m_data[11] = pose[2];

    float w[3];
    w[0] = pose[3];
    w[1] = pose[4];
    w[2] = pose[5];
    const float thetaSquare = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
    const float theta = sqrt(thetaSquare);

    if (theta < sqrtPrecision)
    {
        m_data[0] = 1;
        m_data[1] = 0;
        m_data[2] = 0;

        m_data[4] = 0;
        m_data[5] = 1;
        m_data[6] = 0;

        m_data[8] = 0;
        m_data[9] = 0;
        m_data[10] = 1;
    }
    else
    {
        const float invTheta = 1.0f / theta;

        const float ct = cos(theta);
        const float st = sin(theta);

        const float ux = invTheta * w[0];
        const float uy = invTheta * w[1];
        const float uz = invTheta * w[2];

        m_data[0] = (ct - 1)*uy*uy + (ct - 1)*uz*uz + 1;
        m_data[1] = -st*uz - ux*uy*(ct - 1);
        m_data[2] = st*uy - ux*uz*(ct - 1);

        m_data[4] = st*uz - ux*uy*(ct - 1);
        m_data[5] = (ct - 1)*ux*ux + (ct - 1)*uz*uz + 1;
        m_data[6] = -st*ux - uy*uz*(ct - 1);

        m_data[8] = -st*uy - ux*uz*(ct - 1);
        m_data[9] = st*ux - uy*uz*(ct - 1);
        m_data[10] = (ct - 1)*ux*ux + (ct - 1)*uy*uy + 1;
    }

    m_data[12] = 0;
    m_data[13] = 0;
    m_data[14] = 0;
    m_data[15] = 1;
}

void PoseMatrix4f::rotationFromQuaternion(Quaternion& q)  // check
{
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/

    float sqw = q.w*q.w;
    float sqx = q.x*q.x;
    float sqy = q.y*q.y;
    float sqz = q.z*q.z;

    // invs (inverse square length) is only required if quaternion is not already normalised
    float invs = 1.0f / (sqx + sqy + sqz + sqw);
    m_data[0] = (sqx - sqy - sqz + sqw)
                *invs; // since sqw + sqx + sqy + sqz =1/invs*invs
    m_data[5] = (-sqx + sqy - sqz + sqw)*invs;
    m_data[10] = (-sqx - sqy + sqz + sqw)*invs;

    float tmp1 = q.x*q.y;
    float tmp2 = q.z*q.w;
    m_data[4] = 2.0f * (tmp1 + tmp2)*invs;
    m_data[1] = 2.0f * (tmp1 - tmp2)*invs;

    tmp1 = q.x*q.z;
    tmp2 = q.y*q.w;
    m_data[8] = 2.0f * (tmp1 - tmp2)*invs;
    m_data[2] = 2.0f * (tmp1 + tmp2)*invs;
    tmp1 = q.y*q.z;
    tmp2 = q.x*q.w;
    m_data[9] = 2.0f * (tmp1 + tmp2)*invs;
    m_data[6] = 2.0f * (tmp1 - tmp2)*invs;
}

void PoseMatrix4f::GetTranslation(float* pfTrans)
{
    pfTrans[0] = m_data[3];
    pfTrans[1] = m_data[7];
    pfTrans[2] = m_data[11];
}

void PoseMatrix4f::GetRotation(float* pfRot)
{
    pfRot[0] = m_data[0];
    pfRot[1] = m_data[1];
    pfRot[2] = m_data[2];

    pfRot[3] = m_data[4];
    pfRot[4] = m_data[5];
    pfRot[5] = m_data[6];

    pfRot[6] = m_data[8];
    pfRot[7] = m_data[9];
    pfRot[8] = m_data[10];
}

void PoseMatrix4f::SetRotation(float* pfRot)
{
    m_data[0] = pfRot[0];
    m_data[1] = pfRot[1];
    m_data[2] = pfRot[2];

    m_data[4] = pfRot[3];
    m_data[5] = pfRot[4];
    m_data[6] = pfRot[5];

    m_data[8] = pfRot[6];
    m_data[9] = pfRot[7];
    m_data[10] = pfRot[8];
}

void PoseMatrix4f::SetTranslation(float* pfTrans)
{
    m_data[3] = pfTrans[0];
    m_data[7] = pfTrans[1];
    m_data[11] = pfTrans[2];
}

PoseMatrix4f::~PoseMatrix4f()
{

}

PoseMatrix4f PoseMatrix4f::Inverse() const
{
    return PoseMatrix4f(m_data[0], m_data[4], m_data[8],
                        -((m_data[0] * m_data[3]) + (m_data[4] * m_data[7]) + (m_data[8] * m_data[11])),
                        m_data[1], m_data[5], m_data[9],
                        -((m_data[1] * m_data[3]) + (m_data[5] * m_data[7]) + (m_data[9] * m_data[11])),
                        m_data[2], m_data[6], m_data[10],
                        -((m_data[2] * m_data[3]) + (m_data[6] * m_data[7]) + (m_data[10] *
                                m_data[11])));
}

PoseMatrix4f::PoseMatrix4f(const Matrix4f& poseMat)
{
    SP_copy(poseMat.m_data, poseMat.m_data + 12, m_data, 12);

    m_data[12] = 0.0f;
    m_data[13] = 0.0f;
    m_data[14] = 0.0f;
    m_data[15] = 1.0f;
}

PoseMatrix4f& PoseMatrix4f::operator=(const Matrix4f& poseMat)
{
    if (this != &poseMat && poseMat.m_data != nullptr)
    {
        SP_copy(poseMat.m_data, poseMat.m_data + 12, m_data, 12);

        m_data[12] = 0.0f;
        m_data[13] = 0.0f;
        m_data[14] = 0.0f;
        m_data[15] = 1.0f;
    }
    return *this;
}

bool PoseMatrix4f::operator!=(const PoseMatrix4f &pose) const
{
    return !((*this) == pose);
}

bool PoseMatrix4f::operator==(const PoseMatrix4f &pose) const
{
    static const float epsilon = 0.00001f;
    assert(m_data[12] == 0.0f && m_data[13] == 0.0f && m_data[14] == 0.0f
           && m_data[15] == 1.0f);
    assert(pose.m_data[12] == 0.0f && pose.m_data[13] == 0.0f
           && pose.m_data[14] == 0.0f && pose.m_data[15] == 1.0f);

    for (int i = 0; i < 12; ++i)
    {
        if (std::abs(m_data[i] - pose.m_data[i]) > epsilon)
        {
            return false;
        }
    }
    return true;
}

int PoseMatrix4f::validatePoseMatrix(const float(&poseMatrix)[12])
{
    return PoseMatrix4f::validatePoseMatrix(PoseMatrix4f(poseMatrix));
}

int PoseMatrix4f::validatePoseMatrix(const PoseMatrix4f& pose)
{
    const float *poseMatrix = &pose.m_data[0];

    float3 row1(poseMatrix[0], poseMatrix[1], poseMatrix[2]);
    float3 row2(poseMatrix[4], poseMatrix[5], poseMatrix[6]);
    float3 row3(poseMatrix[8], poseMatrix[9], poseMatrix[10]);
    float3 col1(poseMatrix[0], poseMatrix[4], poseMatrix[8]);
    float3 col2(poseMatrix[1], poseMatrix[5], poseMatrix[9]);
    float3 col3(poseMatrix[2], poseMatrix[6], poseMatrix[10]);

    // matrix is orthogonal : Tranpose(A) * A = I
    bool isValid = true;
    isValid = isValid && (abs(col1.dot(col1) - 1) < PRECISION);
    isValid = isValid && (abs(col1.dot(col2)) < PRECISION);
    isValid = isValid && (abs(col1.dot(col3)) < PRECISION);
    isValid = isValid && (abs(col2.dot(col2) - 1) < PRECISION);
    isValid = isValid && (abs(col2.dot(col3)) < PRECISION);
    isValid = isValid && (abs(col3.dot(col3) - 1) < PRECISION);

    float determinant =
        poseMatrix[0] * (poseMatrix[5] * poseMatrix[10] - poseMatrix[6] * poseMatrix[9])
        -
        poseMatrix[1] * (poseMatrix[4] * poseMatrix[10] - poseMatrix[6] * poseMatrix[8])
        +
        poseMatrix[2] * (poseMatrix[4] * poseMatrix[9] - poseMatrix[5] * poseMatrix[8]);

    //determinant is 1
    isValid = isValid && (abs(determinant - 1) < PRECISION);

    //last row is 0, 0, 0, 1
    isValid = isValid && (std::abs(poseMatrix[12]) < PRECISION
                          && std::abs(poseMatrix[13]) < PRECISION &&
                          std::abs(poseMatrix[14]) < PRECISION
                          && std::abs(poseMatrix[15] - 1.0f) < PRECISION);
    return (isValid ? 1 : 0);
}

Matrix4f::~Matrix4f(){}

bool Matrix4f::operator==(const Matrix4f &matrix) const
{
    for (int i = 0; i < 16; ++i)
    {
        if (m_data[i] != matrix.m_data[i])
        {
            return false;
        }
    }
    return true;
}

BoundingBox::BoundingBox()
{

}

BoundingBox::BoundingBox(float v) :m_minPt(v), m_maxPt(v)
{

}

BoundingBox::BoundingBox(const BoundingBox &bbox):m_minPt(bbox.m_minPt),
    m_maxPt(bbox.m_maxPt)
{

}

BoundingBox::BoundingBox(const float4& minpt,
                         const float4& maxpt):m_minPt(minpt), m_maxPt(maxpt)
{

}

BoundingBox& BoundingBox::operator=(const BoundingBox& bbox)
{
    if (this != &bbox)
    {
        m_minPt = bbox.m_minPt;
        m_maxPt = bbox.m_maxPt;
    }
    return *this;
}

//needed?
Quaternion Quaternion::FromExponential(float3 exp)
{
    float len = exp.length();
    if (len > 1E-5f)
    {
        return FromAxisAngle(float4(exp.x / len, exp.y / len, exp.z / len, len));
    }
    else
    {
        return FromAxisAngle(float4(1, 0, 0, 0));
    }
}

Quaternion Quaternion::Inverse() const
{
    return Quaternion(-x, -y, -z, w);
}

Quaternion Quaternion::operator*(const Quaternion& rightQ)
{
    float3 leftXYZ(*this);
    float3 rightXYZ(rightQ);
    return Quaternion(leftXYZ * rightQ.w + rightXYZ * w + cross(leftXYZ, rightXYZ),
                      w * rightQ.w - leftXYZ.dot(rightXYZ));
}

Quaternion Quaternion::operator*(const float& alpha)
{
    return Quaternion(alpha*x, alpha*y, alpha*z, alpha*w);
}

float Quaternion::Length()
{
    return std::sqrt(w*w + x*x + y*y + z*z);
}

//needed?
Quaternion Quaternion::FromAxisAngle(float4 aa)
{
    float sina = sin(aa.w*0.5f);
    return Quaternion(aa.x * sina, aa.y * sina, aa.z*sina, cos(aa.w*0.5f));
}

Quaternion::Quaternion(float x, float y, float z, float w) :float4(x, y, z, w)
{
}

Quaternion::Quaternion(const float3& xyz, const float w) : float4(xyz.x, xyz.y,
            xyz.z, w)
{

}

Quaternion::Quaternion() : float4()
{

}

Quaternion::Quaternion(const Quaternion& q) : float4(q.x, q.y, q.z, q.w)
{

}

Quaternion::Quaternion(const float4& q) : float4(q.x, q.y, q.z, q.w)
{

}

Quaternion& Quaternion::operator=(const Quaternion& q)
{
    x = q.x;
    y = q.y;
    z = q.z;
    w = q.w;

    return *this;
}

// needed?
// Does not handle Inf, NaN, etc.
float ScenePerception::half2float(short h)
{
    int exponent = (h >> 10) & 0x1F;
    if (exponent != 0)
    {
        exponent += 112; // -15 + 127;
    }
    else if ((h << 1) != 0) //mantissa = 0
    {
        exponent = 113; // - 14 + 127
    }

    int s = ((h & 0x8000) << 16) | (exponent << 23) | ((h & 0x03FF) << 13);

    return (float &)s;
}

bool CorXYZRef::operator==(const CorXYZRef &other) const
{
    return  (std::abs(x - other.x) < 1.0e-5) &&
            (std::abs(y - other.y) < 1.0e-5) &&
            (std::abs(z - other.z) < 1.0e-5);
}

float CorXYZRef::getX() const
{
    return x;
}

float CorXYZRef::getY() const
{
    return y;
}

float CorXYZRef::getZ() const
{
    return z;
}

dim3 ScenePerception::getFitImageBlock(const dim3 &defaultBlock, int width,
                                       int height)
{
    // calculate the exact mapping between threads and pixels.
    // starting from the default block
    int dimenX = defaultBlock.x;
    // check for a value divisible by width, starting from dimenX
    for (int i = 0; i < width - dimenX; ++i)
    {
        // check the value i up
        if (width % (dimenX + i) == 0)
        {
            dimenX = dimenX + i;
            break;
        }
        // check the value i down
        if ((dimenX - i) > 1 && width % (dimenX - i) == 0)
        {
            dimenX = dimenX - i;
            break;
        }
    }

    // check for a value divisible by height, starting from dimenY
    int dimenY = defaultBlock.y;

    for (int i = 0; i < height - dimenY; ++i)
    {
        // check the value i up
        if (height % (dimenY + i) == 0)
        {
            dimenY = dimenY + i;
            break;
        }
        // check the value i down
        if ((dimenY - i) > 1 && height % (dimenY - i) == 0)
        {
            dimenY = dimenY - i;
            break;
        }
    }
    return dim3(dimenX, dimenY);
}
