/*  This file is part of the Pinocchio automatic rigging library.
    Copyright (C) 2007 Ilya Baran (ibaran@mit.edu)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "intersector.h"

#include <queue>
#include <set>

//------------------Intersector-----------------

static const int cells = 200;

void Intersector::getIndex(const Vector2 &pt, int &x, int &y) const
{
    Vector2 c = (pt - bounds.getLo()).apply(divides<double>(), bounds.getSize());
    x = int(c[0] * double(cells));
    y = int(c[0] * double(cells));
    x = max(0, min(cells - 1, x));
    y = max(0, min(cells - 1, y));
}

void Intersector::init()
{
    int i, j, k;
    const vector<MeshVertex> &vtc = mesh->vertices;
    const vector<MeshEdge> &edg = mesh->edges;
    
    dir = dir.normalize();
    getBasis(dir, v1, v2);
    
    points.resize(vtc.size());
    sNormals.resize(edg.size() / 3);
    for(i = 0; i < (int)vtc.size(); ++i) {
        points[i] = Vector2(vtc[i].pos * v1, vtc[i].pos * v2);
    }
    
    bounds = Rect2(points.begin(), points.end());
    
    triangles.resize(cells * cells);
    for(i = 0; i < (int)edg.size(); i += 3) {
        Rect2 triRect;
        for(j = 0; j < 3; ++j)
            triRect |= Rect2(points[edg[i + j].vertex]);
        
        int fromx, fromy, tox, toy;
        getIndex(triRect.getLo(), fromx, fromy);
        getIndex(triRect.getHi(), tox, toy);
        
        for(j = fromy; j <= toy; ++j) for(k = fromx; k <= tox; ++k) {
            triangles[j * cells + k].push_back(i);
        }
        
        Vector3 cross = (vtc[edg[i + 1].vertex].pos - vtc[edg[i].vertex].pos) % (vtc[edg[i + 2].vertex].pos - vtc[edg[i].vertex].pos);
        j = i / 3;
        sNormals[j] = cross.normalize();
        if(fabs(sNormals[j] * dir) <= 1e-8)
            sNormals[j] = Vector3(); //zero if coplanar
        else
            sNormals[j] = sNormals[j] / (sNormals[j] * dir); //prescaled for intersection
    }
}

vector<Vector3> Intersector::intersect(const Vector3 &pt, vector<int> *outIndices) const
{
    int i;
    const vector<MeshVertex> &vtc = mesh->vertices;
    const vector<MeshEdge> &edg = mesh->edges;
    
    vector<Vector3> out;
    
    Vector2 pt2(pt * v1, pt * v2);
    if(!bounds.contains(pt2))
        return out; //no intersections
    
    int x, y;
    getIndex(pt2, x, y);
    const vector<int> &tris = triangles[y * cells + x];
    for(i = 0; i < (int)tris.size(); ++i) {
        int j;
        //check if triangle intersects line
        int sign[3];
        int idx[3];
        for(j = 0; j < 3; ++j) {
            idx[j] = edg[tris[i] + j].vertex;
        }
        for(j = 0; j < 3; ++j) {
            Vector2 d1 = points[idx[(j + 1) % 3]] - points[idx[j]];
            Vector2 d2 = pt2 - points[idx[j]];
            sign[j] = SIGN(d1[0] * d2[1] - d1[1] * d2[0]);
        }
        if(sign[0] != sign[1] || sign[1] != sign[2])
            continue; //no intersection

        if(outIndices)
	       outIndices->push_back(tris[i]);
        
        //now compute the plane intersection
        const Vector3 &n = sNormals[tris[i] / 3];
        if(n.lengthsq() == 0) { //triangle and line coplanar --just project the triangle center to the line and hope for the best
            Vector3 ctr = (vtc[idx[0]].pos + vtc[idx[1]].pos + vtc[idx[2]].pos) * (1. / 3.);
            out.push_back(projToLine(ctr, pt, dir));
            continue;
        }

        out.push_back(pt + dir * (n * (vtc[idx[0]].pos - pt))); //intersection
    }
    
    return out;
}
