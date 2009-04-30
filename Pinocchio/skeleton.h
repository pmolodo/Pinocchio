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

#ifndef SKELETON_H
#define SKELETON_H

#include <map>
#include "graphutils.h"

class PINOCCHIO_API Skeleton {
public:
    Skeleton() {}
    
    const PtGraph &fGraph() const { return fGraphV; }
    const vector<int> &fPrev() const { return fPrevV; }
    const vector<int> &fSym() const { return fSymV; }
    
    const PtGraph &cGraph() const { return cGraphV; }
    const vector<int> &cPrev() const { return cPrevV; }
    const vector<int> &cSym() const { return cSymV; }
    const vector<bool> &cFeet() const { return cFeetV; }
    const vector<bool> &cFat() const { return cFatV; }
    
    const vector<int> &cfMap() const { return cfMapV; }
    const vector<int> &fcMap() const { return fcMapV; }
    const vector<double> &fcFraction() const { return fcFractionV; }
    const vector<double> &cLength() const { return cLengthV; }

    int getJointForName(const std::string &name) const { if(jointNames.count(name)) return jointNames.find(name)->second; return -1; }
    
    void scale(double factor);
    
protected:
    void initCompressed();
    
    //help for creation
    map<string, int> jointNames;
    void makeJoint(const string &name, const Vector3 &pos, const string &previous = string());
    void makeSymmetric(const string &name1, const string &name2);
    void setFoot(const string &name);
    void setFat(const string &name);
    
private:
     //full
    PtGraph fGraphV;
    vector<int> fPrevV; //previous vertices
    vector<int> fSymV; //symmetry
    
    //compressed (no degree 2 vertices)
    PtGraph cGraphV; 
    vector<int> cPrevV; //previous vertices
    vector<int> cSymV; //symmetry
    vector<bool> cFeetV; //whether the vertex should be near the ground
    vector<bool> cFatV; //whether the vertex should be in a large region
    
    vector<int> cfMapV; //compressed to full map
    vector<int> fcMapV; //full to compressed map, -1 when vertex is not in compressed
    vector<double> fcFractionV; //maps full vertex number to ratio of its prev edge length to total length of
                                //containing edge in the compressed graph
    vector<double> cLengthV; //lengths of the compressed bones
};

class PINOCCHIO_API HumanSkeleton : public Skeleton
{
public:
    HumanSkeleton();
};

class PINOCCHIO_API QuadSkeleton : public Skeleton
{
public:
    QuadSkeleton();
};

class PINOCCHIO_API HorseSkeleton : public Skeleton
{
public:
    HorseSkeleton();
};

class PINOCCHIO_API CentaurSkeleton : public Skeleton
{
public:
    CentaurSkeleton();
};

class PINOCCHIO_API FileSkeleton : public Skeleton
{
public:
    FileSkeleton(const std::string &filename);
}; 

#endif
