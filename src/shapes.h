// Jelly shape templates: particles for the soft-body solver (lattice inside an
// implicit surface), shape-matching clusters, and a render mesh skinned to the
// particles. Everything is generated procedurally at startup.
#pragma once
#include <array>
#include <string>
#include <vector>
#include "vmath.h"

enum ShapeKind {
    SHAPE_CUBE,
    SHAPE_SPHERE,
    SHAPE_BEAR,
    SHAPE_WORM,
    SHAPE_HEART,
    SHAPE_STAR,
    SHAPE_CYLINDER,
    SHAPE_PYRAMID,
    SHAPE_COUNT
};

struct Cluster {
    std::vector<int> members;  // particle indices within the body
    Vector3 restCenter;        // centroid of members' rest positions (unscaled)
};

struct ShapeTemplate {
    std::string name;
    float spacing = 0.15f;              // lattice spacing at scale 1
    std::vector<Vector3> rest;          // particle rest positions, centroid at origin
    std::vector<Cluster> clusters;      // local clusters + one global cluster (last)
    float boundRadius = 0.6f;           // max distance of any mesh vertex from origin

    // Render mesh (rest pose, centroid at origin)
    std::vector<Vector3> meshRest;
    std::vector<unsigned short> indices;
    // Skinning: vertex = sum(w_k * x[idx_k]) + R * (scale * offset)
    std::vector<std::array<int, 4>> skinIdx;
    std::vector<std::array<float, 4>> skinW;
    std::vector<Vector3> skinOffset;
};

void BuildShapeTemplates(std::vector<ShapeTemplate>& out);
