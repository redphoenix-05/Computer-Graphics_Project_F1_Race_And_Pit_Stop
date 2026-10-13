// ============================================================================
//  car.h  -  Hierarchical scene graph, Formula 1 car models, vehicle dynamics
//            and the waypoint-following driver used by AI cars / auto mode
// ============================================================================
#pragma once
#include <memory>
#include <functional>
#include "gfx.h"
#include "track.h"

// ---------------------------------------------------------------------------
//  Scene-graph node (hierarchical modelling): world = parent.world * local
// ---------------------------------------------------------------------------
struct Node {
    mat4 local, world, override_;
    const Mesh* mesh = nullptr;
    const DecalMesh* decal = nullptr;
    vec3 tint{1, 1, 1};
    float emis = 0;
    bool visible = true, hero = true, useOverride = false;
    std::vector<Node*> kids;

    void update(const mat4& parent) {
        world = useOverride ? override_ : parent * local;
        for (Node* k : kids) k->update(world);
    }
    void draw(Renderer& r) const {
        if (!visible) return;
        if (mesh) r.draw(*mesh, world, tint, emis, hero);
        if (decal) r.drawDecal(*decal, world);
        for (const Node* k : kids) k->draw(r);
    }
};
struct NodePool {
    std::vector<std::unique_ptr<Node>> items;
    Node* make(Node* parent = nullptr, const Mesh* mesh = nullptr) {
        items.emplace_back(new Node());
        Node* n = items.back().get();
        n->mesh = mesh;
        if (parent) parent->kids.push_back(n);
        return n;
    }
};

// ---------------------------------------------------------------------------
//  Shared primitive meshes (unit sized, tinted at draw time)
// ---------------------------------------------------------------------------
struct Prims {
    Mesh cube, cyl, sphere, sphereGloss, cone, cubeGloss, cylGloss, ring;
    void build() {
        MeshBuilder b;
        b.box(mat4(), vec3(0.5f), MT(vec3(1), 0.15f, 16)); cube.upload(b);
        b.v.clear(); b.box(mat4(), vec3(0.5f), MT(vec3(1), 0.8f, 64)); cubeGloss.upload(b);
        b.v.clear(); b.cyl(mat4(), 1, 1, 0.5f, 20, MT(vec3(1), 0.15f, 16)); cyl.upload(b);
        b.v.clear(); b.cyl(mat4(), 1, 1, 0.5f, 20, MT(vec3(1), 0.8f, 64)); cylGloss.upload(b);
        b.v.clear(); b.sphere(mat4(), 1.f, 20, 14, MT(vec3(1), 0.15f, 16)); sphere.upload(b);
        b.v.clear(); b.sphere(mat4(), 1.f, 20, 14, MT(vec3(1), 0.9f, 80)); sphereGloss.upload(b);
        b.v.clear(); b.cyl(mat4(), 1, 0.f, 0.5f, 20, MT(vec3(1), 0.1f, 16)); cone.upload(b);
    }
};

