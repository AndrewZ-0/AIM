#pragma once

#include "rusty.h"

/*
child   cube rel origin  
0       x+, y+, z+   
1       x+, y+, z-
2       x+, y-, z+
3       x+, y-, z-
4       x-, y+, z+
5       x-, y+, z-
6       x-, y-, z+
7       x-, y-, z-
*/

struct Node {
    Node* children[8];
    Node* parent;
    f64 m;   //mass of subtree
    R3 pos;  //center of mass of subtree
};

struct Octree {
    Node* root;
    f64 scale; //radius of full octree
};

inline bool is_leaf(const Node* n) {
    for (int i = 0; i < 8; i++) {
        if (n->children[i]) return false;
    }
    return true;
}

inline usize choose_octant(const R3& pos, const R3& center) {
    return (
        4 * (pos.x > center.x) + 
        2 * (pos.y > center.y) + 
        (pos.z > center.z)
    );
}

inline void insert_child(Node* root, const R3& pos, f64 mass, f64 rel_scale) {
    Node* curr = root;
    R3 cube_center = root->pos;

    while (true) {
        curr->pos = curr->m * curr->pos + mass * pos;
        curr->m += mass;
        curr->pos /= curr->m;

        usize child_i = choose_octant(pos, cube_center);

        if (!curr->children[child_i]) {
            Node* leaf = new Node{{nullptr}, nullptr, mass, pos};
            leaf->parent = curr;
            curr->children[child_i] = leaf;
            return;
        }

        Node* next = curr->children[child_i];

        rel_scale /= 2;
        cube_center.x += (pos.x > cube_center.x ? 1 : -1) * rel_scale;
        cube_center.y += (pos.y > cube_center.y ? 1 : -1) * rel_scale;
        cube_center.z += (pos.z > cube_center.z ? 1 : -1) * rel_scale;

        if (is_leaf(next)) {
            const usize old_i = choose_octant(next->pos, cube_center);
            Node* new_node = new Node{{nullptr}, next, next->m, next->pos};
            next->children[old_i] = new_node;
        }

        curr = next;
    }
}

inline void push_node(Octree& o, const R3& pos, f64 mass) {
    if (!o.root) {
        o.root = new Node{{nullptr}, nullptr, mass, pos};
        return;
    }

    insert_child(o.root, pos, mass, o.scale);
}

inline Node* find_leaf(const Node* n, const R3& pos, const R3& center, f64 rel_scale) {
    const Node* curr = n;
    R3 c = center;
    
    while (!is_leaf(curr)) {
        usize child_i = choose_octant(pos, c);
        Node* child = curr->children[child_i];
        if (!child) return nullptr;
        
        rel_scale /= 2;
        c.x += (pos.x > c.x ? 1 : -1) * rel_scale;
        c.y += (pos.y > c.y ? 1 : -1) * rel_scale;
        c.z += (pos.z > c.z ? 1 : -1) * rel_scale;
        curr = child;
    }
    return (Node*)curr;
}

inline void update_particle(Octree& o, const R3& old_pos, const R3& pos, f64 mass) {
    if (!o.root) return;
    
    Node* old_leaf = find_leaf(o.root, old_pos, o.root->pos, o.scale);
    if (!old_leaf) return;
    
    const usize old_octant = choose_octant(old_pos, o.root->pos);
    const usize new_octant = choose_octant(pos, o.root->pos);
    
    if (old_octant == new_octant && old_leaf->pos == old_pos) {
        R3 delta = pos - old_leaf->pos;
        old_leaf->pos = pos;
        
        Node* curr = old_leaf->parent;
        while (curr) {
            curr->pos += (mass / curr->m) * delta;
            curr = curr->parent;
        }
    } else {
        Node* curr = old_leaf->parent;
        Node* child_to_remove = old_leaf;
        R3 rem_pos = old_leaf->pos;

        while (curr) {
            const f64 M_old = curr->m;
            const f64 M_new = M_old - mass;
            if (M_new > 0.0) {
                R3 tmp = M_old * curr->pos - mass * rem_pos;
                tmp /= M_new;
                curr->pos = tmp;
            } else {
                curr->pos = {0.0, 0.0, 0.0};
            }
            curr->m = M_new;

            for (int i = 0; i < 8; i++) {
                if (curr->children[i] == child_to_remove) {
                    curr->children[i] = nullptr;
                    break;
                }
            }

            child_to_remove = curr;
            curr = curr->parent;
        }

        delete old_leaf;
        insert_child(o.root, pos, mass, o.scale);
    }
}