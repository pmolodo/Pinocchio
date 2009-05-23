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

#ifndef ATTACHMENT_H
#define ATTACHMENT_H

#include "mesh.h"
#include "skeleton.h"
#include "transform.h"

class VisibilityTester
{
public:
    virtual ~VisibilityTester() {}
    virtual bool canSee(const Vector3 &v1, const Vector3 &v2) const = 0;
};

template<class T> class VisTester : public VisibilityTester
{
public:
    VisTester(const T *t) : tree(t) {}

    virtual bool canSee(const Vector3 &v1, const Vector3 &v2) const //faster when v2 is farther inside than v1
    {
        const double maxVal = 0.002;
        double atV2 = tree->locate(v2)->evaluate(v2);
        double left = (v2 - v1).length();
        double leftInc = left / 100.;
        Vector3 diff = (v2 - v1) / 100.;
        Vector3 cur = v1 + diff;
        while(left >= 0.) {
            double curDist = tree->locate(cur)->evaluate(cur);
            if(curDist > maxVal)
                return false;
            //if curDist and atV2 are so negative that distance won't reach above maxVal, return true
            if(curDist + atV2 + left <= maxVal)
                return true;
            cur += diff;
            left -= leftInc;
        }
        return true;
    }

private:
    const T *tree;
};
template<class T> VisibilityTester *makeVisibilityTester(const T *tree) { return new VisTester<T>(tree); } //be sure to delete afterwards

class AttachmentPrivate;

class PINOCCHIO_API Attachment
{
public:
    Attachment() : a(NULL) {}
    Attachment(const Attachment &);
    Attachment(const Mesh &mesh, const Skeleton &skeleton, const vector<Vector3> &match, const VisibilityTester *tester, double initialHeatWeight=1.);
    virtual ~Attachment();

    Mesh deform(const Mesh &mesh, const vector<Transform<> > &transforms) const;
    Vector<double, -1> getWeights(int i) const;
private:
    AttachmentPrivate *a;
};

#endif
