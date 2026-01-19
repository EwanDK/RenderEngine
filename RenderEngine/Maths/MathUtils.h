#pragma once
#include <limits>
#define _USE_MATH_DEFINES
#include <math.h>

inline float toRadians(float degrees){
    return degrees * M_PI / 180.f;
}

inline float toDegrees(float radians){
    return radians * 180.f / M_PI;
}

inline bool nearZero(float val, float epsilon = 0.001f){
    return fabs(val)<=epsilon;
}

inline float cotan(float angle){
    return 1.0f / tanf(angle);
}

inline void GenerateUvSphere(int rings,int slices,float radius,float** outVertices,float** outNormals,unsigned short** outIndices,int* outVertexCount,int* outIndexCount){
    
    const int vertexCount = (rings + 1) * (slices + 1);
    const int indexCount  = rings * slices * 6;

    float* vertices = (float*)malloc(vertexCount * 3 * sizeof(float));
    float* normals  = (float*)malloc(vertexCount * 3 * sizeof(float));
    unsigned short* indices = (unsigned short*)malloc(indexCount * sizeof(unsigned short));

    int v = 0;

    //Vertices, normals
    for (int y = 0; y <= rings; y++){
        
        float vRatio = (float)y / (float)rings;
        float theta = vRatio * M_PI;  // 0 → PI

        float sinTheta = sinf(theta);
        float cosTheta = cosf(theta);

        for (int x = 0; x <= slices; x++){
            
            float uRatio = (float)x / (float)slices;
            float phi = uRatio * 2.0f * M_PI; // 0 → 2PI

            float sinPhi = sinf(phi);
            float cosPhi = cosf(phi);

            float nx = cosPhi * sinTheta;
            float ny = cosTheta;
            float nz = sinPhi * sinTheta;

            // Position
            vertices[v * 3 + 0] = radius * nx;
            vertices[v * 3 + 1] = radius * ny;
            vertices[v * 3 + 2] = radius * nz;

            // Normal
            normals[v * 3 + 0] = nx;
            normals[v * 3 + 1] = ny;
            normals[v * 3 + 2] = nz;

            v++;
        }
    }

    //Indices
    int i = 0;
    for (int y = 0; y < rings; y++){
        
        for (int x = 0; x < slices; x++){
            
            int i0 = y * (slices + 1) + x;
            int i1 = i0 + slices + 1;
            int i2 = i0 + 1;
            int i3 = i1 + 1;

            // Triangle 1
            indices[i++] = i0;
            indices[i++] = i1;
            indices[i++] = i2;

            // Triangle 2
            indices[i++] = i2;
            indices[i++] = i1;
            indices[i++] = i3;
        }
    }

    *outVertices = vertices;
    *outNormals = normals;
    *outIndices = indices;
    *outVertexCount = vertexCount;
    *outIndexCount = indexCount;
}

class Matrix3
{
public:
    float mat[3][3];

    Matrix3(){
        *this = Matrix3::identity;
    }

    explicit Matrix3(float inMat[3][3]){
        memcpy(mat, inMat, 9 * sizeof(float));
    }

	// Cast to a const float pointer
	const float* getAsFloatPtr() const{
    	return reinterpret_cast<const float*>(&mat[0][0]);
    }

    static const Matrix3 identity;

    Vector3 getColumn(int col) const{
        return Vector3(mat[0][col], mat[1][col], mat[2][col]);
    }

    void setColumn(int col, const Vector3& v){
        mat[0][col] = v.x;
        mat[1][col] = v.y;
        mat[2][col] = v.z;
    }

    // Matrix multiplication
    friend Matrix3 operator*(const Matrix3& a, const Matrix3& b){
        Matrix3 result;
        for (int i = 0; i < 3; ++i) {       // row
            for (int j = 0; j < 3; ++j) {   // column
                result.mat[i][j] =
                    a.mat[i][0] * b.mat[0][j] +
                    a.mat[i][1] * b.mat[1][j] +
                    a.mat[i][2] * b.mat[2][j];
            }
        }
        return result;
    }

    Matrix3& operator*=(const Matrix3& right){
        *this = *this * right;
        return *this;
    }

    // Orthonormalize columns (Gram-Schmidt)
    Matrix3 orthonormalized() const{
        Matrix3 result;

        Vector3 x = Vector3Normalize(getColumn(0));
        Vector3 y = Vector3Normalize(getColumn(1) - x * Vector3DotProduct(getColumn(1),x));
        Vector3 z = Vector3CrossProduct(x, y); // ensures a right-handed basis

        result.setColumn(0, x);
        result.setColumn(1, y);
        result.setColumn(2, z);

        return result;
    }

    // Outer product of two vectors: returns a matrix
    static Matrix3 outerProduct(const Vector3& a, const Vector3& b){
        Matrix3 result;
        result.mat[0][0] = a.x * b.x;
        result.mat[0][1] = a.x * b.y;
        result.mat[0][2] = a.x * b.z;

        result.mat[1][0] = a.y * b.x;
        result.mat[1][1] = a.y * b.y;
        result.mat[1][2] = a.y * b.z;

        result.mat[2][0] = a.z * b.x;
        result.mat[2][1] = a.z * b.y;
        result.mat[2][2] = a.z * b.z;

        return result;
    }

    Matrix3& operator+=(const Matrix3& other){
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                mat[r][c] += other.mat[r][c];
        return *this;
    }

    void zero(){
        memset(mat, 0, 9 * sizeof(float));
    }

	Matrix3 polarDecomposition() const {
    	// Placeholder: just orthonormalize columns using Gram-Schmidt
    	Matrix3 R = *this;
    	R=R.orthonormalized();
    	return R;
    }
};

const Matrix3 Matrix3::identity = [] {
    Matrix3 m;
    m.mat[0][0] = 1.0f;
    m.mat[0][1] = 0.0f;
    m.mat[0][2] = 0.0f;

    m.mat[1][0] = 0.0f;
    m.mat[1][1] = 1.0f;
    m.mat[1][2] = 0.0f;

    m.mat[2][0] = 0.0f;
    m.mat[2][1] = 0.0f;
    m.mat[2][2] = 1.0f;
    return m;
}();