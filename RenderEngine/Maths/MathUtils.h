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
