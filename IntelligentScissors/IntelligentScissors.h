//
//  IntelligentScissors.h
//  DoctorAssitant
//
//  Created by Chao Li on 8/8/14.
//  Copyright (c) 2014 Luis Alvarado. All rights reserved.
//

#ifndef __DoctorAssitant__IntelligentScissors__
#define __DoctorAssitant__IntelligentScissors__

#include <iostream>
#include "fibheap.h"
using namespace cv;

class PixelNode:public FibHeapNode
{
public:
    enum State{INITIAL, ACTIVE, EXPANDED};
private:
    double linkCost[8];
    int column, row;
public:
    PixelNode(int c,int r);
    void setLinkCost(int link,double value);
    double LinkCost(int link);//return linkcost with linkID
    double LinkCost(PixelNode *pn);//return linkcost between this and pn, return -1 if they are not neighbor;this is not used now
    State state;
    double totalCost;
    PixelNode *prevNode;
    double Column();
    double Row();
    virtual void operator =(FibHeapNode& RHS);
    virtual int  operator ==(FibHeapNode& RHS);
    virtual int  operator <(FibHeapNode& RHS);
    virtual void Print();
    void Neighbor(int link,int &c,int &r);//return column and row of neighbor in link direction
    
};

class QPoint{
public:
    int row;
    int column;
    QPoint(int c,int r);
};

class Iscissor
{
private:
    Mat grayImageMat;
    Mat seedOriented;
    Mat normalizedGrayImage;
    Mat LapBeforeScale;
    //Mat dirMat;
    int row,column;
    Mat gradX,gradY,gradXY,insides,outsides;
    vector<double> edgeTraining,gradTraining,insideTraining,outsideTraining;
    vector<vector<PixelNode*>> pixelNodes;
    void costFunctionPaper();
    void costFunctionModify();
    double getD(int i,int j,int link);
    void updatePathTree();
    void init_PixelNodes();
    void init_Image(const Mat& image);
    void deleteAllNodes();
    void computeSides(int dist);
    cv::Point unitGradVector(int c,int r);
    void calculateTraining(vector<QPoint> trainingPoints,int granularity,Mat input,vector<double> output);
    void gaussianBlur(vector<double> buffer,vector<double> output);
    int getTrainingIdx(int granularity,double value);
    double getTrainedEdge(double edge);
    double getTrainedGrad(double grad);
    double getTrainedInside(double inside);
    double getTrainedOutside(double outside);
    double gradDirection(PixelNode* ori,PixelNode*nei);
    ~Iscissor();
public:
    Iscissor(Mat& image);
    PixelNode *seed;
    vector<vector<int>> stabilty;
    vector<vector<QPoint>> outline;
    QPoint coordinateTransfer(QPoint matpoint);
    void setImage(Mat& image);
    void setSeed(int row,int column);
    QPoint snapSeed(int column,int row);
    void getPath(int column, int row, vector<QPoint> &path);
    void doTraing(vector<QPoint> points);
    double dist(PixelNode *ori,PixelNode *neighbor);
};

#endif /* defined(__DoctorAssitant__IntelligentScissors__) */
