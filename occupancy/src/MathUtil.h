#pragma once
#include <vector>
#include <ostream>

typedef float cl_float;
struct cl_float2
{
    float s[2];
};
struct cl_float3
{
    float s[4];
};
struct cl_float4
{
    float s[4];
};

typedef int cl_int;
struct cl_int2
{
    int s[2];
};
struct cl_int3
{
    int s[4];
};
struct cl_int4
{
    int s[4];
};

typedef unsigned int cl_uint;
struct cl_uint2
{
    unsigned int s[2];
};
struct cl_uint3
{
    unsigned int s[4];
};
struct cl_uint4
{
    unsigned int s[4];
};

typedef short cl_short;
struct cl_short2
{
    short s[2];
};
struct cl_short3
{
    short s[4];
};
struct cl_short4
{
    short s[4];
};

typedef unsigned short cl_ushort;
struct cl_ushort2
{
    unsigned short x, y;
};
struct cl_ushort3
{
    unsigned short x, y, z, w;
};
struct cl_ushort4
{
    unsigned short x, y, z, w;
};

typedef unsigned char cl_uchar;
struct cl_uchar2
{
    unsigned char x, y;
};
struct cl_uchar3
{
    unsigned char x, y, z, w;
};
struct cl_uchar4
{
    unsigned char s[4];
};

#include <algorithm>
#include <cmath>
#include <chrono>

#undef min
#undef max

namespace ScenePerception
{
inline size_t RoundToWorkgroupPower(size_t size, size_t wrk)  // check
{
    return (size%wrk == 0) ? size : wrk*(size / wrk + 1);
}
class Matrix4f;

const float PRECISION = 0.000005f;
const float PI = 3.14159265f;

template<class T>
inline T clamp(const T& x, const T& l, const T& h)
{
    return (std::max)((std::min)(x, h), l);
}

//! <summary>
//! iVector4 is templated to accommodate different vector types of 04 components and 03 components.
//! \param OCL_T4: the corresponding OpenCL type (e.g. cl_float4, cl_float3)
//! \param T : the corresponding scalar type for components of the OpenCL type (e.g. cl_float, cl_char)
//! \param is4Mem : indicates if it is a vector of 04 components (if true) or 03 components otherwise.
//! Note: the implementation does not handle calculation values exceeding their type range (MIN, MAX).
//! Use AsFloat() to convert to the corresponding floating type (e.g. vector of char, short) at the cost of precision/performance.
//!</summary>
template<typename T, typename OCL_T4, bool is4Mem>
class iVector4
{
public:
    iVector4() :x(0), y(0), z(0),
        w(0) {} //compiler currently does not warn of uninitialized/unassigned struct/class

    iVector4(T _a) :x(_a), y(_a), z(_a)
    {
        w = (is4Mem ? x : 0);
    }

    //if construction with 04 values is used for a vector of 03 components, the 4th value is ignored
    // and w is set to zero.
    iVector4(T _x, T _y, T _z, T _w = 0) :x(_x), y(_y), z(_z)
    {
        w = (is4Mem ? _w : 0);
    }

    // OpenCL vector convention puts z, w = 0
    iVector4(T _a, T _b) :x(_a), y(_b), z(0), w(0) {};

    iVector4(const OCL_T4 &other):x(other.s[0]), y(other.s[1]),
        z(other.s[2])
    {
        //set the w to 0 if the wrapper is for vector 3.
        w = (is4Mem ? (other.s[3]):0);
    }

    iVector4(const iVector4<T, OCL_T4, true> &other) :x(other.x), y(other.y),
        z(other.z)
    {
        w = (is4Mem ? other.w : 0);
    }

    iVector4(const iVector4<T, OCL_T4, false> &other) :x(other.x), y(other.y),
        z(other.z), w(0) {}

    //return a float copy of the vector.
    iVector4<cl_float, cl_float4, is4Mem> AsFloat() const
    {
        return iVector4<cl_float, cl_float4, is4Mem>(static_cast<float>(x),
                static_cast<float>(y), static_cast<float>(z),
                (is4Mem ? static_cast<float>(w) : 0));
    }

    iVector4& operator=(const iVector4 &rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        w = (is4Mem ? rhs.w : w);
        return *this;
    }

    iVector4& operator=(T a)
    {
        x = a;
        y = a;
        z = a;
        w = (is4Mem ? a : w);
        return *this;
    }

    iVector4& operator+=(const iVector4 &rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        w = (is4Mem ? w + rhs.w : w);
        return *this;
    }

    iVector4& operator+=(T a)
    {
        x += a;
        y += a;
        z += a;
        w = (is4Mem ? w + a : w);
        return *this;
    }

    iVector4 operator+(const iVector4 &rhs) const
    {
        return iVector4<T, OCL_T4, is4Mem>(*this) += rhs;
    }

    iVector4 operator+(T a) const
    {
        return iVector4<T, OCL_T4, is4Mem>(*this) += a;
    }

    iVector4& operator-=(const iVector4 &rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        w = (is4Mem ? w - rhs.w : w);
        return *this;
    }

    iVector4& operator-=(T a)
    {
        x -= a;
        y -= a;
        z -= a;
        w = (is4Mem ? w - a : w);
        return *this;
    }

    iVector4 operator-(const iVector4 &rhs) const
    {
        return iVector4<T, OCL_T4, is4Mem>(*this) -= rhs;
    }

    iVector4 operator-(T a) const
    {
        return iVector4<T, OCL_T4, is4Mem>(*this) -= a;
    }

    iVector4& operator*=(const iVector4 &rhs)
    {
        x *= rhs.x;
        y *= rhs.y;
        z *= rhs.z;
        w = (is4Mem ? w *rhs.w : w);
        return *this;
    }

    iVector4& operator*=(T a)
    {
        x *= a;
        y *= a;
        z *= a;
        w = (is4Mem ? w * a : w);
        return *this;
    }

    iVector4 operator*(const iVector4 &rhs) const
    {
        return iVector4<T, OCL_T4, is4Mem>(*this) *= rhs;
    }

    iVector4 operator*(T a) const
    {
        return iVector4<T, OCL_T4, is4Mem>(*this) *= a;
    }

    iVector4& operator/=(const iVector4 &rhs)
    {
        x /= rhs.x;
        y /= rhs.y;
        z /= rhs.z;
        w = (is4Mem ? w / rhs.w : w);
        return *this;
    }

    iVector4& operator/=(T a)
    {
        x /= a;
        y /= a;
        z /= a;
        w = (is4Mem ? w / a : w);
        return *this;
    }

    iVector4 operator/(const iVector4 &rhs) const
    {
        return iVector4<T, OCL_T4, is4Mem>(*this) /= rhs;
    }

    iVector4 operator/(T a) const
    {
        return iVector4<T, OCL_T4, is4Mem>(*this) /= a;
    }

    bool operator==(const iVector4<T, OCL_T4, is4Mem> &rhs) const
    {
        return (x == rhs.x) && (y == rhs.y)
               && (z == rhs.z) && (is4Mem? (w == rhs.w):true);
    }

    bool operator==(T a) const
    {
        return (x == a) && (y == a)
               && (z == a) && (is4Mem? (w == a):true);
    }

    bool operator!=(const iVector4<T, OCL_T4, is4Mem> &rhs) const
    {
        return !(*this == rhs);
    }

    bool operator!=(T a) const
    {
        return !(*this == a);
    }

    //! normalize does not cover cases where the vector length is <= 1E-16f
    //! nor for integer/short, etc vectors
    iVector4& normalized()
    {
        float len = length();
        if (len > 1E-16f)
        {
            len = 1.0f / len;
            x = static_cast<T>(x * len);
            y = static_cast<T>(y * len);
            z = static_cast<T>(z * len);
            w = (is4Mem ? static_cast<T>(w * len) : w);
        }
        return *this;
    }

    iVector4 normalize() const
    {
        return iVector4<T, OCL_T4, is4Mem>(*this).normalized();
    }

    float length() const
    {
        return static_cast<float>(sqrt(x * x + y * y + z * z +
                                       (is4Mem ? (w * w):0)));
    }

    float distance(const iVector4<T, OCL_T4, is4Mem> &pt2) const
    {
        return static_cast<float>(sqrt((x - pt2.x) * (x - pt2.x) +
                                       (y - pt2.y) * (y - pt2.y) +
                                       (z - pt2.z) * (z - pt2.z) +
                                       (is4Mem?(w - pt2.w)*(w - pt2.w):0)));
    }

    float distance3D(const iVector4<T, OCL_T4, is4Mem> &pt2) const
    {
        return static_cast<float>(sqrt((x - pt2.x) * (x - pt2.x) +
                                       (y - pt2.y) * (y - pt2.y) +
                                       (z - pt2.z) * (z - pt2.z)));
    }

    float squaredDistance3D(const iVector4<T, OCL_T4, is4Mem> &pt2) const
    {
        return static_cast<float>((x - pt2.x) * (x - pt2.x) +
                                  (y - pt2.y) * (y - pt2.y) +
                                  (z - pt2.z) * (z - pt2.z));
    }

    float dot(const iVector4<T, OCL_T4, is4Mem> &rhs) const
    {
        return static_cast<float>(x * rhs.x + y * rhs.y + z * rhs.z +
                                  (is4Mem ? (w * rhs.w) : 0));
    }

    iVector4& Mix(const iVector4<T, OCL_T4, is4Mem> &v1, float alpha)
    {
        x += static_cast<T>(alpha * (v1.x - x));
        y += static_cast<T>(alpha * (v1.y - y));
        z += static_cast<T>(alpha * (v1.z - z));
        w = (is4Mem? static_cast<T>(w + alpha * (v1.w - w)):0);
        return *this;
    }

    iVector4& Min(const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        x = std::min(x, rhs.x);
        y = std::min(y, rhs.y);
        z = std::min(z, rhs.z);
        w = (is4Mem?std::min(w, rhs.w):w);
        return *this;
    };

    iVector4& Max(const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        x = std::max(x, rhs.x);
        y = std::max(y, rhs.y);
        z = std::max(z, rhs.z);
        w = (is4Mem?std::max(w, rhs.w):w);
        return *this;
    };

    friend iVector4 operator*(T a, const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        return iVector4<T, OCL_T4, is4Mem>(a) *= rhs;
    };

    friend iVector4 operator+(T a, const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        return iVector4<T, OCL_T4, is4Mem>(a) += rhs;
    };

    friend iVector4 operator-(T a, const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        return iVector4<T, OCL_T4, is4Mem>(a) -= rhs;
    };

    friend iVector4 operator/(T a, const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        return iVector4<T, OCL_T4, is4Mem>(a) /= rhs;
    };

    friend bool operator==(T a, const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        return rhs == a;
    };

    friend bool operator!=(T a, const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        return !(rhs == a);
    };

    friend iVector4 mix(const iVector4<T, OCL_T4, is4Mem> &lhs,
                        const iVector4<T, OCL_T4, is4Mem> &rhs, float alpha)
    {
        return iVector4<T, OCL_T4, is4Mem>(lhs).Mix(rhs, alpha);
    }

    friend iVector4 min(const iVector4<T, OCL_T4, is4Mem> &lhs,
                        const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        return iVector4<T, OCL_T4, is4Mem>(lhs).Min(rhs);
    };

    friend iVector4 max(const iVector4<T, OCL_T4, is4Mem> &lhs,
                        const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        return iVector4<T, OCL_T4, is4Mem>(lhs).Max(rhs);
    };

    friend float dot(const iVector4<T, OCL_T4, is4Mem> &lhs,
                     const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        return lhs.dot(rhs);
    }

    //! cross function operates only on the first three components of vector (regardless of having
    //! 4 or 3 components)
    friend iVector4<float, cl_float4, false> cross(const iVector4<T, OCL_T4, is4Mem>
            &lhs,
            const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        return iVector4<float, cl_float4, false>(
                   static_cast<float>(lhs.y * rhs.z - lhs.z * rhs.y),
                   static_cast<float>(lhs.z * rhs.x - lhs.x * rhs.z),
                   static_cast<float>(lhs.x * rhs.y - lhs.y * rhs.x));
    }

    template<typename charT, typename traits>
    friend std::basic_ostream<charT, traits> &
    operator<< (std::basic_ostream<charT, traits> &stream, const iVector4 &rhs)
    {
        stream << "(" << rhs.x << "," << rhs.y << "," << rhs.z << ",";
        if (is4Mem)
        {
            stream << rhs.w;
        }
        stream << ")";
        return stream;
    }

    //interface with OpenCL type - OCL_T4
    operator OCL_T4() const
    {
        OCL_T4 result;
        result.s[0] = x;
        result.s[1] = y;
        result.s[2] = z;
        result.s[3] = (is4Mem ? w : 0);
        return result;
    }

    //data members
    T x;
    T y;
    T z;
    T w;
};

//specialization for particular types
template<>
bool iVector4<float, cl_float4, true>::operator==(const
        iVector4<float, cl_float4, true> &rhs) const;

template<>
bool iVector4<float, cl_float4, true>::operator==(float a) const;

template<>
bool iVector4<float, cl_float4, false>::operator==(const
        iVector4<float, cl_float4, false> &rhs) const;

template<>
bool iVector4<float, cl_float4, false>::operator==(float a) const;

template class iVector4<float, cl_float4, false>;
template class iVector4<float, cl_float4, true>;
template class iVector4<short, cl_short4, true>;
template class iVector4<int, cl_int4, false>;
template class iVector4<int, cl_int4, true>;
template class iVector4<unsigned char, cl_uchar4, true>;
typedef iVector4<float, cl_float4, true> float4;
typedef iVector4<short, cl_short4, true> short4;
typedef iVector4<int, cl_int4, true> int4;

//WARNING: these unsigned types are not tested for operations that involve value overflow
typedef iVector4<unsigned int, cl_uint4, false> uint3;
typedef iVector4<unsigned char, cl_uchar4, true> uchar4;

template<bool is4Mem>
inline iVector4<unsigned char, cl_uchar4, is4Mem> scaleRange(
    const iVector4<float, cl_float4, true> &rgb)
{
    return iVector4<unsigned char, cl_uchar4, is4Mem>(
               static_cast<unsigned char>(clamp(rgb.x*255.0f, 0.0f, 255.0f)),
               static_cast<unsigned char>(clamp(rgb.y*255.0f, 0.0f, 255.0f)),
               static_cast<unsigned char>(clamp(rgb.z*255.0f, 0.0f, 255.0f)),
               static_cast<unsigned char>(clamp(rgb.w*255.0f, 0.0f, 255.0f)));
}

template<bool is4Mem>
inline iVector4<float, cl_float4, is4Mem> scaleRange(const
        iVector4<unsigned char, cl_uchar4, is4Mem> &rgb)
{
    return iVector4<float, cl_float4, true>(rgb.x / 255.0f, rgb.y / 255.0f,
                                            rgb.z / 255.0f, (is4Mem ? (rgb.w / 255.0f) : 1.0f));
}

template<typename T, typename OCL_T4>
class Vector3
{
public:
    Vector3() :x(0), y(0), z(
            0) {} //compiler currently does not warn of uninitialized/unassigned struct/class

    Vector3(T _x, T _y, T _z) :x(_x), y(_y), z(_z) {}

    Vector3(const OCL_T4 &other) :x(other.s[0]), y(other.s[1]), z(other.s[2]) {}

    Vector3(const Vector3<T, OCL_T4> &other) :x(other.x), y(other.y), z(other.z) {}

    //return a float copy of the vector.
    Vector3<cl_float, cl_float4> AsFloat() const
    {
        return Vector3<cl_float, cl_float4>(
                   static_cast<float>(x),
                   static_cast<float>(y),
                   static_cast<float>(z));
    }

    Vector3& operator=(const Vector3 &rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }

    //interface with vector of 04 components
    template<bool is4Mem>
    Vector3& operator=(const iVector4<T, OCL_T4, is4Mem> &rhs)
    {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }

    Vector3& operator=(T a)
    {
        x = a;
        y = a;
        z = a;
        return *this;
    }

    Vector3& operator+=(const Vector3 &rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vector3& operator+=(T a)
    {
        x += a;
        y += a;
        z += a;
        return *this;
    }

    Vector3 operator+(const Vector3 &rhs) const
    {
        return Vector3<T, OCL_T4>(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    Vector3 operator+(T a) const
    {
        return Vector3<T, OCL_T4>(x + a, y + a, z + a);
    }

    Vector3& operator-=(const Vector3 &rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vector3& operator-=(T a)
    {
        x -= a;
        y -= a;
        z -= a;
        return *this;
    }

    Vector3 operator-(const Vector3 &rhs) const
    {
        return Vector3<T, OCL_T4>(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    Vector3 operator-(T a) const
    {
        return Vector3<T, OCL_T4>(x - a, y - a, z - a);
    }

    Vector3& operator*=(const Vector3 &rhs)
    {
        x *= rhs.x;
        y *= rhs.y;
        z *= rhs.z;
        return *this;
    }

    Vector3& operator*=(T a)
    {
        x *= a;
        y *= a;
        z *= a;
        return *this;
    }

    Vector3 operator*(const Vector3 &rhs) const
    {
        return Vector3<T, OCL_T4>(x * rhs.x, y * rhs.y, z * rhs.z);
    }

    Vector3 operator*(T a) const
    {
        return Vector3<T, OCL_T4>(x * a, y * a, z * a);
    }

    Vector3& operator/=(const Vector3 &rhs)
    {
        x /= rhs.x;
        y /= rhs.y;
        z /= rhs.z;
        return *this;
    }

    Vector3& operator/=(T a)
    {
        x /= a;
        y /= a;
        z /= a;
        return *this;
    }

    Vector3 operator/(const Vector3 &rhs) const
    {
        return Vector3<T, OCL_T4>(x / rhs.x, y / rhs.y, z / rhs.z);
    }

    Vector3 operator/(T a) const
    {
        return Vector3<T, OCL_T4>(x / a, y / a, z / a);
    }

    bool operator==(const Vector3<T, OCL_T4> &rhs) const
    {
        return (x == rhs.x) && (y == rhs.y)
               && (z == rhs.z);
    }

    bool operator==(T a) const
    {
        return (x == a) && (y == a)
               && (z == a);
    }

    bool operator!=(const Vector3<T, OCL_T4> &rhs) const
    {
        return !(*this == rhs);
    }

    bool operator!=(T a) const
    {
        return !(*this == a);
    }

    //! normalize does not cover cases where the vector length is <= 1E-16f
    //! nor for integer/short, etc vectors
    Vector3& normalized()
    {
        float len = length();
        if (len > 1E-16f)
        {
            len = 1.0f / len;
            x = static_cast<T>(x * len);
            y = static_cast<T>(y * len);
            z = static_cast<T>(z * len);
        }
        return *this;
    }

    Vector3 normalize() const
    {
        float len = length();
        if (len > 1E-16f)
        {
            len = 1.0f / len;
            return Vector3<T, OCL_T4>(
                       static_cast<T>(x * len),
                       static_cast<T>(y * len),
                       static_cast<T>(z * len));
        }
        else
        {
            return *this;
        }
    }

    float length() const
    {
        return static_cast<float>(sqrt(x * x + y * y + z * z));
    }

    float squaredLength() const
    {
        return static_cast<float>(x * x + y * y + z * z);
    }

    float distance(const Vector3<T, OCL_T4> &pt2) const
    {
        return static_cast<float>(sqrt(
                                      (x - pt2.x) * (x - pt2.x) +
                                      (y - pt2.y) * (y - pt2.y) +
                                      (z - pt2.z) * (z - pt2.z)));
    }

    float dot(const Vector3<T, OCL_T4> &rhs) const
    {
        return static_cast<float>(x * rhs.x + y * rhs.y + z * rhs.z);
    }

    Vector3& Mix(const Vector3<T, OCL_T4> &v1, float alpha)
    {
        x += static_cast<T>(alpha * (v1.x - x));
        y += static_cast<T>(alpha * (v1.y - y));
        z += static_cast<T>(alpha * (v1.z - z));
        return *this;
    }

    Vector3& Min(const Vector3<T, OCL_T4> &rhs)
    {
        x = std::min(x, rhs.x);
        y = std::min(y, rhs.y);
        z = std::min(z, rhs.z);
        return *this;
    };

    Vector3& Max(const Vector3<T, OCL_T4> &rhs)
    {
        x = std::max(x, rhs.x);
        y = std::max(y, rhs.y);
        z = std::max(z, rhs.z);
        return *this;
    };

    friend Vector3 operator*(T a, const Vector3<T, OCL_T4> &rhs)
    {
        return Vector3<T, OCL_T4>(a * rhs.x, a * rhs.y, a * rhs.z);
    };

    friend Vector3 operator+(T a, const Vector3<T, OCL_T4> &rhs)
    {
        return Vector3<T, OCL_T4>(a + rhs.x, a + rhs.y, a + rhs.z);
    };

    friend Vector3 operator-(T a, const Vector3<T, OCL_T4> &rhs)
    {
        return Vector3<T, OCL_T4>(a - rhs.x, a - rhs.y, a - rhs.z);
    };

    friend Vector3 operator/(T a, const Vector3<T, OCL_T4> &rhs)
    {
        return Vector3<T, OCL_T4>(a / rhs.x, a / rhs.y, a / rhs.z);
    };

    friend bool operator==(T a, const Vector3<T, OCL_T4> &rhs)
    {
        return rhs == a;
    };

    friend bool operator!=(T a, const Vector3<T, OCL_T4> &rhs)
    {
        return !(rhs == a);
    };

    friend Vector3 mix(const Vector3<T, OCL_T4> &lhs, const Vector3<T, OCL_T4> &rhs,
                       float alpha)
    {
        return Vector3<T, OCL_T4>(
                   static_cast<T>(lhs.x + alpha * (rhs.x - lhs.x)),
                   static_cast<T>(lhs.y + alpha * (rhs.y - lhs.y)),
                   static_cast<T>(lhs.z + alpha * (rhs.z - lhs.z)));
    }

    friend Vector3 min(const Vector3<T, OCL_T4> &lhs,
                       const Vector3<T, OCL_T4> &rhs)
    {
        return Vector3<T, OCL_T4>(
                   std::min(lhs.x, rhs.x),
                   std::min(lhs.y, rhs.y),
                   std::min(lhs.z, rhs.z));
    };

    friend Vector3 max(const Vector3<T, OCL_T4> &lhs,
                       const Vector3<T, OCL_T4> &rhs)
    {
        return Vector3<T, OCL_T4>(
                   std::max(lhs.x, rhs.x),
                   std::max(lhs.y, rhs.y),
                   std::max(lhs.z, rhs.z));
    };

    friend float dot(const Vector3<T, OCL_T4> &lhs, const Vector3<T, OCL_T4> &rhs)
    {
        return lhs.dot(rhs);
    }

    friend OCL_T4 & assign(OCL_T4 &lhs, const Vector3<T, OCL_T4> &rhs)
    {
        lhs.s[0] = rhs.x;
        lhs.s[1] = rhs.y;
        lhs.s[2] = rhs.z;
        lhs.s[3] = 0;
        return lhs;
    }

    //! cross function operates only on the first three components of vector (regardless of having
    //! 4 or 3 components)
    friend Vector3<float, cl_float4> cross(const Vector3<T, OCL_T4> &lhs,
                                           const Vector3<T, OCL_T4> &rhs)
    {
        return Vector3<float, cl_float4>(
                   static_cast<float>(lhs.y * rhs.z - lhs.z * rhs.y),
                   static_cast<float>(lhs.z * rhs.x - lhs.x * rhs.z),
                   static_cast<float>(lhs.x * rhs.y - lhs.y * rhs.x));
    }

    template<typename charT, typename traits>
    friend std::basic_ostream<charT, traits> &
    operator<< (std::basic_ostream<charT, traits> &stream, const Vector3 &rhs)
    {
        stream << "(" << rhs.x << "," << rhs.y << "," << rhs.z << "," << ")";
        return stream;
    }

    //interface with OpenCL type - OCL_T4
    operator OCL_T4() const
    {
        OCL_T4 result;
        result.s[0] = x;
        result.s[1] = y;
        result.s[2] = z;
        result.s[3] = 0;
        return result;
    }

    //data members
    T x;
    T y;
    T z;
protected:
    //prevent automatic construction by compiler that involves an overhead in
    // operation with scalar. For intential object construction, should call Vector3(a, b, c) instead.
    Vector3(T _a) :x(_a), y(_a), z(_a) {};
};

//specialization for particular types
template<>
bool Vector3<float, cl_float4>::operator==(const Vector3<float, cl_float4> &rhs)
const;

template<>
bool Vector3<float, cl_float4>::operator==(float a) const;

template class Vector3<float, cl_float4>;
template class Vector3<int, cl_int4>;
template class Vector3<int, cl_uchar4>;
typedef class Vector3<float, cl_float4> float3;
typedef class Vector3<int, cl_int4> int3;
typedef class Vector3<int, cl_uchar4> uchar3;


//type of vector of 2 components
//! Note: several functions (dot, distance, length, etc) internally use float values of data members
//! for the calculations to avoid overflow (e.g. vector of char, short) at the cost of precision/performance.
template<typename T, typename OCL_T2>
class iVector2
{
public:
    iVector2() :x(0), y(0) {};

    iVector2(T _x, T _y) :x(_x), y(_y) {};

    iVector2(const iVector2 &other): x(other.x), y(other.y)
    {
    }

    iVector2<cl_float, cl_float2> AsFloat() const
    {
        return iVector2<cl_float, cl_float2>(static_cast<float>(x),
                                             static_cast<float>(y));
    }

    iVector2& operator=(const iVector2 &rhs)
    {
        x = rhs.x;
        y = rhs.y;
        return *this;
    }

    iVector2& operator=(T a)
    {
        x = a;
        y = a;
        return *this;
    }

    iVector2& operator+=(const iVector2 &rhs)
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    iVector2& operator+=(T a)
    {
        x += a;
        y += a;
        return *this;
    }

    iVector2 operator+(const iVector2 &rhs) const
    {
        return iVector2<T, OCL_T2>(*this) += rhs;
    }

    iVector2 operator+(T a) const
    {
        return iVector2<T, OCL_T2>(*this) += a;
    }

    iVector2& operator-=(const iVector2 &rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    iVector2& operator-=(T a)
    {
        x -= a;
        y -= a;
        return *this;
    }

    iVector2 operator-(const iVector2 &rhs) const
    {
        return iVector2<T, OCL_T2>(*this) -= rhs;
    }

    iVector2 operator-(T a) const
    {
        return iVector2<T, OCL_T2>(*this) -= a;
    }

    iVector2& operator*=(const iVector2 &rhs)
    {
        x *= rhs.x;
        y *= rhs.y;
        return *this;
    }

    iVector2& operator*=(T a)
    {
        x *= a;
        y *= a;
        return *this;
    }

    iVector2 operator*(const iVector2 &rhs) const
    {
        return iVector2<T, OCL_T2>(*this) *= rhs;
    }

    iVector2 operator*(T a) const
    {
        return iVector2<T, OCL_T2>(*this) *= a;
    }

    iVector2& operator/=(const iVector2 &rhs)
    {
        x /= rhs.x;
        y /= rhs.y;
        return *this;
    }

    iVector2& operator/=(T a)
    {
        x /= a;
        y /= a;
        return *this;
    }

    iVector2 operator/(const iVector2 &rhs) const
    {
        return iVector2<T, OCL_T2>(*this) /= rhs;
    }

    iVector2 operator/(T a) const
    {
        return iVector2<T, OCL_T2>(*this) /= a;
    }

    bool operator==(const iVector2 &rhs) const
    {
        return (x == rhs.x) && (y == rhs.y);
    }

    bool operator==(T a) const
    {
        return (x == a) && (y == a);
    }

    bool operator!=(const iVector2 &rhs) const
    {
        return !(*this == rhs);
    }

    bool operator!=(T a) const
    {
        return !(*this == a);
    }

    //! normalize does not cover cases where the vector length is <= 1E-16f
    //! nor for integer/short, etc vectors
    iVector2& normalized()
    {
        float len = length();
        if (len > 1E-16f)
        {
            len = 1.0f / len;
            x = static_cast<T>(x * len);
            y = static_cast<T>(y * len);
        }
        return *this;
    }

    iVector2 normalize()
    {
        return iVector2<T, OCL_T2>(*this).normalized();
    }

    float length() const
    {
        return sqrt(static_cast<float>(x * x + y * y));
    }

    float distance(const iVector2 &pt2) const
    {
        return sqrt(static_cast<float>((x - pt2.x) * (x - pt2.x) +
                                       (y - pt2.y) * (y - pt2.y)));
    }

    float dot(const iVector2 &rhs) const
    {
        return static_cast<float>(x * rhs.x + y * rhs.y);
    }

    iVector2 &Mix(const iVector2 &v1, float alpha)
    {
        x += static_cast<T>(alpha * (v1.x - x));
        y += static_cast<T>(alpha * (v1.y - y));
        return *this;
    }

    iVector2 &Min(const iVector2 &rhs)
    {
        x = std::min(x, rhs.x);
        y = std::min(y, rhs.y);
        return *this;
    };

    iVector2 &Max(const iVector2 &rhs)
    {
        x = std::max(x, rhs.x);
        y = std::max(y, rhs.y);
        return *this;
    };

    friend iVector2 operator*(T a, const iVector2 &rhs)
    {
        return iVector2<T, OCL_T2>(a) *= rhs;
    }

    friend iVector2 operator+(T a, const iVector2 &rhs)
    {
        return iVector2<T, OCL_T2>(a) += rhs;
    };

    friend iVector2 operator-(T a, const iVector2 &rhs)
    {
        return iVector2<T, OCL_T2>(a) -= rhs;
    };

    friend iVector2 operator/(T a, const iVector2 &rhs)
    {
        return iVector2<T, OCL_T2>(a) /= rhs;
    };

    friend bool operator==(T a, const iVector2 &rhs)
    {
        return rhs == a;
    };

    friend bool operator!=(T a, const iVector2 &rhs)
    {
        return !(rhs == a);
    };

    friend iVector2 mix(const iVector2 &lhs, const iVector2 &rhs, float alpha)
    {
        return iVector2<T, OCL_T2>(lhs).Mix(rhs, alpha);
    }

    friend iVector2 min(const iVector2 &lhs, const iVector2 &rhs)
    {
        return iVector2<T, OCL_T2>(lhs).Min(rhs);
    };

    friend iVector2 max(const iVector2 &lhs, const iVector2 &rhs)
    {
        return iVector2<T, OCL_T2>(lhs).Max(rhs);
    };

    friend float dot(const iVector2 &lhs, const iVector2 &rhs)
    {
        return iVector2<T, OCL_T2>(lhs).dot(rhs);
    }

    template<typename charT, typename traits>
    friend std::basic_ostream<charT, traits> &
    operator<< (std::basic_ostream<charT, traits> &lhs, const iVector2 &rhs)
    {
        lhs << "(" << rhs.x << "," << rhs.y << ")";
        return lhs;
    }

    operator OCL_T2() const
    {
        OCL_T2 result;
        result.s[0] = x;
        result.s[1] = y;
        return result;
    }

    //data member
    T x;
    T y;
protected:
    //prevent automatic construction by compiler that involves an overhead in
    // operation with scalar. For intential object construction, should call iVector2(a, b) instead.
    iVector2(T _a) :x(_a), y(_a) {};
};

//specialization for particular types
template<>
bool iVector2<float, cl_float2>::operator==(const iVector2<float, cl_float2>
        &rhs) const;

template<>
bool iVector2<float, cl_float2>::operator==(float a) const;

template class iVector2<float, cl_float2>;
template class iVector2<unsigned int, cl_uint2>;
template class iVector2<int, cl_int2>;
template class iVector2<short, cl_short2>;
typedef iVector2<float, cl_float2> float2;
typedef iVector2<unsigned int, cl_uint2> uint2;
typedef iVector2<int, cl_int2> int2;
typedef iVector2<short, cl_short2> short2;

class Quaternion : public float4
{
public:
    Quaternion();
    Quaternion(const Quaternion& q);
    Quaternion(const float4& q);
    Quaternion(float x, float y, float z, float w);
    Quaternion(const float3& xyz, const float w);

    Quaternion& operator=(const Quaternion& q);
    Quaternion operator*(const Quaternion& rightQ);
    Quaternion operator*(const float& alpha);

    void Conjugate();
    Quaternion Inverse() const;
    void Normalize();

    float Length();
    float4 ToAxisAngle() const;
    float3 ToExponential();
    float3 ToEulerAngles() const;

    static Quaternion FromAxisAngle(float4 aa);
    static Quaternion FromExponential(float3 exp);
};

//TODO: change to use 08 member array?
#pragma pack(push, 1)
class BoundingBox
{
public:
    float4 m_minPt;
    float4 m_maxPt;
    float4 m_padding1;
    float4 m_padding2;

    //these must be initialized to point to m_data in ALL constructors
    BoundingBox();
    BoundingBox(float v);
    BoundingBox(const BoundingBox &bbox);
    BoundingBox(const float4& minpt, const float4& maxpt);
    BoundingBox& operator=(const BoundingBox& b);

    template<typename charT, typename traits>
    friend std::basic_ostream<charT, traits> &
    operator<< (std::basic_ostream<charT, traits> &lhs, const BoundingBox &rhs)
    {
        lhs << "[" << rhs.m_minPt << ":" << rhs.m_maxPt << "]";
        return lhs;
    }

    bool Intersect(const BoundingBox& tBox) const
    {
        return(this->m_maxPt.x > tBox.m_minPt.x &&
               this->m_minPt.x < tBox.m_maxPt.x &&
               this->m_maxPt.y > tBox.m_minPt.y &&
               this->m_minPt.y < tBox.m_maxPt.y &&
               this->m_maxPt.z > tBox.m_minPt.z &&
               this->m_minPt.z < tBox.m_maxPt.z);
    }
};
#pragma pack(pop)

class Matrix4f
{
public:
    float m_data[16];

    Matrix4f();
    explicit Matrix4f(const float val);
    explicit Matrix4f(const std::vector<float>& vals);
    Matrix4f(float m00, float m01, float m02, float m03,
             float m10, float m11, float m12, float m13,
             float m20, float m21, float m22, float m23,
             float m30, float m31, float m32, float m33);
    explicit Matrix4f(const float(&vals)[16]);
    explicit Matrix4f(const float(&vals)[12]);
    virtual ~Matrix4f();

    static Matrix4f Identity;
    static Matrix4f Zero;

    Matrix4f Transpose() const;
    float Det() const;
    Matrix4f Inverse() const;
    void SetMatrix(const float (&pose)[6]);
    static float4 Transform(const Matrix4f& mat, const float4& vec);
    static float3 Transform(const Matrix4f& M, const float3& vec);

    void Set(const std::vector<float>& M);
    void Get(std::vector<float>& M) const;
    float At(int i, int j) const;
    void Set(int i, int j, float newValue);

    Matrix4f MultiplyWith(const Matrix4f& M) const;

    Matrix4f operator-(const Matrix4f& M) const;
    Matrix4f operator+(const Matrix4f& M) const;
    Matrix4f operator*(const float val) const;

    Matrix4f operator-(const float val) const;
    Matrix4f operator+(const float val) const;
    Matrix4f operator/(const float val) const;
    Matrix4f operator*(const Matrix4f& M) const;
    float4 operator*(const float4& v) const;

    bool operator==(const Matrix4f &matrix) const;
    Matrix4f& operator=(const Matrix4f& M);

    static Matrix4f MakePerspectiveFieldOfView(const float& fovy,
            const float& aspect, const float& zNear, const float& zFar);
    static Matrix4f MakePerspectiveOffCenter(const float& left, const float& right,
            const float& bottom, const float& top, const float& zNear, const float& zFar);
	static Matrix4f MakeUnitVectorRotation(const float4& v1, const float4& v2);

    template<typename charT, typename traits>
    friend std::basic_ostream<charT, traits> &
    operator<< (std::basic_ostream<charT, traits> &lhs, Matrix4f const &rhs)
    {
        lhs << "\n[" << rhs.m_data[0] << " " << rhs.m_data[1] << " " << rhs.m_data[2] <<
            " " << rhs.m_data[3] << "]\n"
            << "[" << rhs.m_data[4] << " " << rhs.m_data[5] << " " << rhs.m_data[6] << " "
            << rhs.m_data[7] << "]\n"
            << "[" << rhs.m_data[8] << " " << rhs.m_data[9] << " " << rhs.m_data[10] << " "
            << rhs.m_data[11] << "]\n"
            << "[" << rhs.m_data[12] << " " << rhs.m_data[13] << " " << rhs.m_data[14] <<
            " " << rhs.m_data[15] << "]\n";
        return lhs;
    }

    friend float4 operator*(const float4 &v, const Matrix4f &matrix)
    {
        return float4(
                   dot(v, float4(matrix.m_data[0], matrix.m_data[4], matrix.m_data[8],
                                 matrix.m_data[12])),
                   dot(v, float4(matrix.m_data[1], matrix.m_data[5], matrix.m_data[9],
                                 matrix.m_data[13])),
                   dot(v, float4(matrix.m_data[2], matrix.m_data[6], matrix.m_data[10],
                                 matrix.m_data[14])),
                   dot(v, float4(matrix.m_data[3], matrix.m_data[7], matrix.m_data[11],
                                 matrix.m_data[15])));
    }
};

class PoseMatrix4f : public Matrix4f
{
public:
    PoseMatrix4f();

    PoseMatrix4f(const PoseMatrix4f &poseMat);

    PoseMatrix4f(const std::vector<float>& vals);

    PoseMatrix4f(float m00, float m01, float m02, float m03,
                 float m10, float m11, float m12, float m13,
                 float m20, float m21, float m22, float m23);
    PoseMatrix4f(const float(&rotationMat)[9], const float3 & translationVector);
    PoseMatrix4f(const float(&rotationMat)[3][3], const float(&translationVector)[3]);
    PoseMatrix4f(const float3& rotationMatRow1, const float3& rotationMatRow2,
                 const float3& rotationMatRow3,
                 const float3& translationVector);
    PoseMatrix4f(const float(&vals)[12]);

    PoseMatrix4f(const float(&vals)[16]);
    PoseMatrix4f(const float(&vals)[6]);
    PoseMatrix4f(const float(&pose)[6], double sqrtPrecision);
    PoseMatrix4f(const Matrix4f& poseMat);
    PoseMatrix4f& operator=(const Matrix4f& poseMat);
    bool operator!=(const PoseMatrix4f &pose) const;
    bool operator==(const PoseMatrix4f &pose) const;
    PoseMatrix4f& operator=(const PoseMatrix4f&);

    ~PoseMatrix4f();

    /// WARNING: several repeated uses may degenerate pose matrix: PoseMatrix4f * PoseMatrix4f.Inverse() <> Identity.
    /// Use Matrix4f instead.
    PoseMatrix4f Inverse() const;

    PoseMatrix4f operator*(const PoseMatrix4f&) const;
    float4 operator*(const float4& v) const;
    float3 operator*(const float3& v) const;

    static int validatePoseMatrix(const float(&poseMatrix)[12]);

    static int validatePoseMatrix(const PoseMatrix4f&);

    void convert6DOFToMatrix(const float(&pose)[6], double sqrtPrecision);

    void convertTo6DOF(float(&pose)[6]);

    Quaternion findRotationQuaternion() const;
    void rotationFromQuaternion(Quaternion& q);

    void SetRotation(float* pfRot);
    void SetTranslation(float* pfTrans);
    void GetRotation(float* pfRot);
    void GetTranslation(float* pfTrans);

    template<typename charT, typename traits>
    friend std::basic_ostream<charT, traits> &
    operator<< (std::basic_ostream<charT, traits> &lhs, PoseMatrix4f const &rhs)
    {
        lhs << "\n[" << rhs.m_data[0] << " " << rhs.m_data[1] << " " << rhs.m_data[2] <<
            " " << rhs.m_data[3] << "]\n"
            << "[" << rhs.m_data[4] << " " << rhs.m_data[5] << " " << rhs.m_data[6] << " "
            << rhs.m_data[7] << "]\n"
            << "[" << rhs.m_data[8] << " " << rhs.m_data[9] << " " << rhs.m_data[10] << " "
            << rhs.m_data[11] << "]\n"
            << "[" << rhs.m_data[12] << " " << rhs.m_data[13] << " " << rhs.m_data[14] <<
            " " << rhs.m_data[15] << "]\n";
        return lhs;
    }
};

Matrix4f GetCameraMatrix(const float4 &kParam);

Matrix4f GetInverseCameraMatrix(const float4 &kParam);

typedef struct Triangle
{
    ScenePerception::float4 p[3];
} Triangle;

typedef struct GridCell
{
    ScenePerception::float3 p[8];
    short val[8];
    unsigned short wei[8];
} GridCell;

typedef ScenePerception::float3(*VertexInterpF) (short isolevel,
        ScenePerception::float3 point1, ScenePerception::float3 point2, short valp1,
        short valp2);

float half2float(short h);

typedef short(*ConvertEle)(short h);

typedef struct CorXYZRef
{
public:
    //!< condition: corArray must have prior memory allocation of at least (idx + 3) number of floats.
    CorXYZRef(float _x, float _y, float _z) :x(_x), y(_y), z(_z) {}

    CorXYZRef& operator=(const CorXYZRef &other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    bool operator==(const CorXYZRef &other) const;

    bool operator<(const CorXYZRef &other) const;

    float getX() const;

    float getY() const;

    float getZ() const;

private:
    CorXYZRef(); //disable default constructor
    float x, y, z;
} CorXYZRef;

//! dim3 holds unsigned integer values (default 1) for dimensions x, y, and z.
typedef struct dim3
{
    dim3(cl_uint _x, cl_uint _y, cl_uint _z) :x(_x), y(_y), z(_z) {}
    dim3(cl_uint _x, cl_uint _y) :x(_x), y(_y), z(1) {}
    dim3(cl_uint _x) : x(_x), y(1), z(1) {}
    dim3() :x(1), y(1), z(1) {}
    cl_uint x, y, z;
} dim3;

dim3 getFitImageBlock(const dim3 &defaultBlock, int width, int height);

}
