/*
Copyright (c) 2007 Ilya Baran

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef DEFMESH_H_INCLUDED
#define DEFMESH_H_INCLUDED

#include "../Pinocchio/attachment.h"
#include "filter.h"
#include "DisplayMesh.h"

class Motion;

class DefMesh : public DisplayMesh
{
public:
    DefMesh(const Mesh inMesh, const Skeleton &inOrigSkel, const vector<Vector3> &inMatch,
            const Attachment &inAttachment, Motion *inMotion = NULL) 
      : origSkel(inOrigSkel), match(inMatch), attachment(inAttachment),
        origMesh(inMesh), motion(inMotion), filter(match, origSkel.fPrev())
    {
        transforms.resize(origSkel.fPrev().size() - 1);
    }

    void setMotion(Motion *inMotion) { motion = inMotion; }
    Motion *getMotion() const { return motion; }
    void updateIfHasMotion() const { if(motion) updateMesh(); }

    vector<Vector3> getSkel() const;
    const Skeleton &getOrigSkel() const { return origSkel; }

    const Attachment &getAttachment() const { return attachment; }

    const Mesh &getMesh() { updateMesh(); return curMesh; }

private:
    double getLegRatio() const;
    vector<Transform<> > computeTransforms() const;
    void updateMesh() const;

    Skeleton origSkel;
    vector<Vector3> match;
    Attachment attachment;
    Mesh origMesh;
    mutable Mesh curMesh;
    vector<Quaternion<> > transforms;
    Motion *motion;

    vector<double> footOffsets;
    mutable MotionFilter filter;
};

#endif //DEFMESH_H_INCLUDED
