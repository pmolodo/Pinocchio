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

#ifndef FILTER_H_INCLUDED
#define FILTER_H_INCLUDED

#include "../Pinocchio/matrix.h"
#include "../Pinocchio/vector.h"
#include "../Pinocchio/transform.h"

class MotionFilter
{
public:
    MotionFilter(const vector<Vector3> &inJoints, const vector<int> inPrev)
        : joints(inJoints), prev(inPrev) {}

    void step(const vector<Transform<> > &transforms, vector<Vector3> feet);

    const vector<Transform<> > &getTransforms() const { return curTransforms; }

private:
    Matrixn<double> getJac(const vector<Transform<> > &transforms) const;
    void addTranslation();

    vector<Vector3> joints;
    vector<int> prev;

    Vector3 prevTrans;
    Vectorn<double> prevFeet;
    vector<Transform<> > curTransforms;
};

#endif //FILTER_H_INCLUDED
