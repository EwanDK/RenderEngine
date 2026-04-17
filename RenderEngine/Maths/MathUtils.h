#pragma once
#include <limits>
#define _USE_MATH_DEFINES
#include <cstdlib>
#include <math.h>
#include <raymath.h>

inline float toRadians(float degrees){
    return degrees * M_PI / 180.f;
}

inline float toDegrees(float radians){
    return radians * 180.f / M_PI;
}

inline bool nearZero(float val, float epsilon = 0.001f){
    return fabs(val)<=epsilon;
}

inline Vector3 SafeNormalize(Vector3 v, Vector3 fallback = {0, 1, 0}){
    float len = Vector3Length(v);
    return (len > 1e-6f) ? v * (1.0f / len) : fallback;
}

inline float cotan(float angle){
    return 1.0f / tanf(angle);
}

inline void GenerateUvSphere(int rings,int slices,float radius,float** outVertices,float** outNormals,unsigned short** outIndices,int* outVertexCount,int* outIndexCount){
    
    // ---- Counts ----
    const int vertexCount = 2 + (rings - 1) * slices;
    const int indexCount = slices * 3 + slices * 6 * (rings - 2) + slices * 3;

    float* vertices = (float*)malloc(vertexCount * 3 * sizeof(float));
    float* normals = (float*)malloc(vertexCount * 3 * sizeof(float));
    unsigned short* indices = (unsigned short*)malloc(indexCount * sizeof(unsigned short));

    if (!vertices || !normals || !indices) {
        // malloc failed
        free(vertices); free(normals); free(indices);
        *outVertices = *outNormals = NULL;
        *outIndices = NULL;
        *outVertexCount = *outIndexCount = 0;
        return;
    }

    int v = 0;

    // ---- Top pole ----
    vertices[0] = 0.0f;
    vertices[1] = radius;
    vertices[2] = 0.0f;

    normals[0] = 0.0f;
    normals[1] = 1.0f;
    normals[2] = 0.0f;

    v = 1;

    //Vertices, normals
    for (int y = 1; y < rings; y++){
        
        float vRatio = (float)y / (float)rings;
        float theta = vRatio * M_PI;  // 0 → PI

        float sinTheta = sinf(theta);
        float cosTheta = cosf(theta);

        for (int x = 0; x < slices; x++){
            
            float uRatio = (float)x / (float)slices;
            float phi = uRatio * 2.0f * M_PI; // 0 → 2PI

            float sinPhi = sinf(phi);
            float cosPhi = cosf(phi);

            float nx = cosPhi * sinTheta;
            float ny = cosTheta;
            float nz = sinPhi * sinTheta;

            // Position
            vertices[v * 3] = radius * nx;
            vertices[v * 3 + 1] = radius * ny;
            vertices[v * 3 + 2] = radius * nz;

            // Normal
            normals[v * 3] = nx;
            normals[v * 3 + 1] = ny;
            normals[v * 3 + 2] = nz;

            v++;
        }
    }

    // ---- Bottom pole ----
    vertices[v*3] = 0.0f;
    vertices[v*3+1] = -radius;
    vertices[v*3+2] = 0.0f;

    normals[v*3] = 0.0f;
    normals[v*3+1] = -1.0f;
    normals[v*3+2] = 0.0f;

    const int bottomIndex = v;

    //Indices
    int i = 0;

    // Top cap
    for (int x = 0; x < slices; x++)
    {
        int a = 0;
        int b = 1 + x;
        int c = 1 + (x + 1) % slices;

        indices[i++] = a;
        indices[i++] = b;
        indices[i++] = c;
    }
    
    // Middle quads
    for (int y = 0; y < rings - 2; y++)
    {
        int ringStart = 1 + y * slices;
        int nextRing  = ringStart + slices;

        for (int x = 0; x < slices; x++)
        {
            int i0 = ringStart + x;
            int i1 = ringStart + (x + 1) % slices;
            int i2 = nextRing + x;
            int i3 = nextRing + (x + 1) % slices;

            indices[i++] = i0;
            indices[i++] = i2;
            indices[i++] = i1;

            indices[i++] = i1;
            indices[i++] = i2;
            indices[i++] = i3;
        }
    }

    // Bottom cap
    int lastRingStart = 1 + (rings - 2) * slices;

    for (int x = 0; x < slices; x++)
    {
        int a = lastRingStart + x;
        int b = bottomIndex;
        int c = lastRingStart + (x + 1) % slices;

        indices[i++] = a;
        indices[i++] = b;
        indices[i++] = c;
    }

    *outVertices = vertices;
    *outNormals = normals;
    *outIndices = indices;
    *outVertexCount = vertexCount;
    *outIndexCount = indexCount;
}

inline void GeneratePlane(float sizeX,float sizeY,int subdivX,int subdivY,Vector3 normal,float** outVertices,float** outNormals,unsigned short** outIndices,int* outVertexCount,int* outIndexCount){

    const int vertexCount = (subdivX+1)*(subdivY+1);
    const int indexCount = subdivX*subdivY*6;

    float* vertices = (float*)malloc(vertexCount * 3 * sizeof(float));
    float* normals = (float*)malloc(vertexCount * 3 * sizeof(float));
    unsigned short* indices = (unsigned short*)malloc(indexCount * sizeof(unsigned short));

    if (!vertices || !normals || !indices) {
        // malloc failed
        *outVertices = *outNormals = NULL;
        *outIndices = NULL;
        *outVertexCount = *outIndexCount = 0;
        return;
    }

    float nx = normal.x;
    float ny = normal.y;
    float nz = normal.z;

    //Tangent and Bitangent from normal, gives a plane of any orientation
    //Helper vector (world up by default)
    float hx, hy, hz;
    if (fabsf(ny) < 0.999f) { hx = 0.0f; hy = 1.0f; hz = 0.0f; }
    else                     { hx = 1.0f; hy = 0.0f; hz = 0.0f; }

    // T = normalize(helper × N)
    float tx = hy*nz - hz*ny;
    float ty = hz*nx - hx*nz;
    float tz = hx*ny - hy*nx;
    float tLen = sqrtf(tx*tx + ty*ty + tz*tz);
    tx /= tLen; ty /= tLen; tz /= tLen;

    // B = normalize(N × T)  — no need to re-normalize if N and T are already unit
    float bx = ny*tz - nz*ty;
    float by = nz*tx - nx*tz;
    float bz = nx*ty - ny*tx;
    // (already unit length since N⊥T, but normalizing anyway for safety)
    float bLen = sqrtf(bx*bx + by*by + bz*bz);
    bx /= bLen; by /= bLen; bz /= bLen;

    // --- Generate vertices ---
    for (int row = 0; row <= subdivY; row++) {
        for (int col = 0; col <= subdivX; col++) {
            int idx = (row * (subdivX + 1) + col) * 3;

            // u, v in [-0.5, 0.5] → centred at origin
            float u = ((float)col / (float)subdivX) - 0.5f;
            float v = ((float)row / (float)subdivY) - 0.5f;

            vertices[idx + 0] = u * sizeX * tx + v * sizeY * bx;
            vertices[idx + 1] = u * sizeX * ty + v * sizeY * by;
            vertices[idx + 2] = u * sizeX * tz + v * sizeY * bz;

            normals[idx + 0] = nx;
            normals[idx + 1] = ny;
            normals[idx + 2] = nz;
        }
    }

    // --- Generate indices (CCW winding) ---
    int ii = 0;
    for (int row = 0; row < subdivY; row++) {
        for (int col = 0; col < subdivX; col++) {
            unsigned short tl = (unsigned short)( row      * (subdivX + 1) + col    );
            unsigned short tr = (unsigned short)( row      * (subdivX + 1) + col + 1);
            unsigned short bl = (unsigned short)((row + 1) * (subdivX + 1) + col    );
            unsigned short br = (unsigned short)((row + 1) * (subdivX + 1) + col + 1);

            // Triangle 0: tl → tr → bl
            indices[ii++] = tl;
            indices[ii++] = tr;
            indices[ii++] = bl;
            // Triangle 1: tr → br → bl
            indices[ii++] = tr;
            indices[ii++] = br;
            indices[ii++] = bl;
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

inline const Matrix3 Matrix3::identity = [] {
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