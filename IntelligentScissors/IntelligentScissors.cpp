//
//  IntelligentScissors.cpp
//  DoctorAssitant
//
//  Created by Chao Li on 8/8/14.
//  Copyright (c) 2014 Luis Alvarado. All rights reserved.
//

#include "IntelligentScissors.h"

const int linktable[8][2]={{1,0},{1,-1},{0,-1},{-1,-1}
    ,{-1,0},{-1,1},{0,1},{1,1}};

double PixelNode::Column(){return column;}

double PixelNode::Row(){return row;}

PixelNode::PixelNode(int c,int r):FibHeapNode(){
    state = INITIAL;
    prevNode = NULL;
    column = c;
    row = r;
    totalCost = DBL_MAX;
}

void PixelNode::operator =(FibHeapNode& RHS)
{
    PixelNode& pRHS=(PixelNode&)RHS;
    FHN_Assign(RHS);
    totalCost=pRHS.totalCost;
}

int PixelNode::operator ==(FibHeapNode& RHS)
{
    PixelNode& pRHS=(PixelNode&)RHS;
    if (FHN_Cmp(RHS)) return 0;
    // Key compare goes here in derived classes
    if(RHS.NegInfinityFlag&&NegInfinityFlag) return 1;
    return totalCost==pRHS.totalCost;
}

int PixelNode::operator <(FibHeapNode& RHS)
{
    int X;
    PixelNode& pRHS=(PixelNode&)RHS;
    if ((X=FHN_Cmp(RHS)) != 0)
        return X < 0 ? 1 : 0;
    // Key compare goes here in derived classes
    if(RHS.NegInfinityFlag&&NegInfinityFlag)
        return 0;
    return totalCost<pRHS.totalCost;
}

void PixelNode::Print()
{
    if (NegInfinityFlag)
        std::cout << "-inf.";
    else std::cout << totalCost;
}

void PixelNode::Neighbor(int link, int &c, int &r)
{
    c=column+linktable[link][0];
    r=row+linktable[link][1];
}

void PixelNode::setLinkCost(int link, double value)
{
    linkCost[link]=value;
}

double PixelNode::LinkCost(int link)
{
    return linkCost[link];
}

double PixelNode::LinkCost(PixelNode *pn)
{
    if(pn->Column()==column+1&&pn->Row()==row)
        return linkCost[0];
    if(pn->Column()==column+1&&pn->Row()==row-1)
        return linkCost[1];
    if(pn->Column()==column&&pn->Row()==row-1)
        return linkCost[2];
    if(pn->Column()==column-1&&pn->Row()==row-1)
        return linkCost[3];
    if(pn->Column()==column-1&&pn->Row()==row)
        return linkCost[4];
    if(pn->Column()==column-1&&pn->Row()==row+1)
        return linkCost[5];
    if(pn->Column()==column&&pn->Row()==row+1)
        return linkCost[6];
    if(pn->Column()==column+1&&pn->Row()==row+1)
        return linkCost[7];
    return -1.0;
}

Iscissor::Iscissor(Mat& image){
    init_Image(image);
    costFunctionPaper();
    int edgeWidth = 2;
    computeSides(edgeWidth);
    //costFunctionModify();
}

void Iscissor::init_Image(const cv::Mat &image){
    grayImageMat = image;
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;
    minMaxLoc(grayImageMat, &minVal, &maxVal, &minLoc, &maxLoc );
    std::cout<<"minVal"<<minVal<<"maxVal"<<maxVal;
    insides = grayImageMat;
    outsides = grayImageMat;
    pixelNodes.resize(grayImageMat.rows);
    stabilty.resize(grayImageMat.rows);
    for (int i=0; i<grayImageMat.rows; i++) {
        for (int j=0; j<grayImageMat.cols; j++) {
            pixelNodes[i].push_back(new PixelNode(j,i));
            stabilty[i].push_back(0);
        }
    }

    seed = NULL;
//    QImage m(img->width(),img->height(),img->format());
//    m.fill(qRgb(255,255,255));
//    mask=m;
    std::cout<<"init image Function Completed";
}


void Iscissor::init_PixelNodes(){
    int h = grayImageMat.rows;
    int w = grayImageMat.cols;
    for (int i=0; i<h; i++) {
        for (int j=0; j<w; j++) {
            pixelNodes[i][j]->totalCost = DBL_MAX;
            pixelNodes[i][j]->state = PixelNode::INITIAL;
            stabilty[i][j]=0;
        }
    }
    seed->totalCost = 0;
    seed->prevNode = NULL;
    std::cout<<"init pixel Function Completed";
}

void Iscissor::setSeed(int row, int col){
    seed = pixelNodes[row][col];
    updatePathTree();
}

void Iscissor::updatePathTree(){
    int seedRow = seed->Row();
    int seedCol = seed->Column();
    int range = 40;
    int top,bottom,right,left;
    seedRow + range >=grayImageMat.rows?bottom=grayImageMat.rows-1:bottom = seedRow+range;
    seedRow - range <0?top = 0:top=seedRow-range;
    seedCol + range >=grayImageMat.cols?right = grayImageMat.cols-1:right = seedCol+range;
    seedCol - range <0?left=0:left=seedCol-range;
//
//    CvRect smallCentreRect = cvRect(left,top,right-left+1,bottom-top+1);
//    seedOriented = grayImageMat(smallCentreRect);
    init_PixelNodes();
    FibHeap heap;
    heap.Insert(seed);
    PixelNode *q;
    while (heap.Minimum()!=NULL) {
        q = (PixelNode *)heap.ExtractMin();
        q->state = PixelNode::EXPANDED;
        int r,c;
        for (int i=0; i<8; i++) {
            q->Neighbor(i, c, r);
            if (c>=left && c<right && r>=top && r<bottom) {
                PixelNode *pn = pixelNodes[r][c];
                if (pn->state != PixelNode::EXPANDED) {
                    q->setLinkCost(i, dist(q, pn));
                    double cost = q->LinkCost(i);
                    //double cost = dist(q, pn);
                    if (pn->state == PixelNode::INITIAL) {
                        pn->prevNode = q;
                        pn->totalCost = cost+q->totalCost;
                        pn->state = PixelNode::ACTIVE;
                        heap.Insert(pn);
                    }
                    else if (pn->state == PixelNode::ACTIVE){
                        if (cost + q->totalCost < pn->totalCost) {
                            pn->prevNode = q;
                            PixelNode newpn(pn->Column(),pn->Row());
                            newpn.totalCost=cost+q->totalCost;
                            heap.DecreaseKey(pn,newpn);
                        }
                    }
                }
            }
        }
    }
    std::cout<<"update path Function Completed";
}

void Iscissor::getPath(int column, int row, vector<QPoint> &path)
{
    PixelNode *pn=pixelNodes[row][column];
    while(pn!=NULL)
    {
        path.push_back(QPoint(pn->Column(),pn->Row()));
        pn=pn->prevNode;
    }
    reverse(path.begin(),path.end());
}

QPoint::QPoint(int c,int r){
    row  =r;
    column = c;
}

QPoint Iscissor::snapSeed(int column, int row){
    Mat edge;
    Canny(grayImageMat, edge, 31,127,3 ,true);
    double m = DBL_MAX;
    int c = column;
    int r = row;
    for (int i=0; i<edge.rows; i++) {
        for (int j=0; j<edge.cols; j++) {
            if(edge.at<uchar>(i,j)!=0)
            {
                double d=(i-row)*(i-row)+(j-column)*(j-column);
                if(d<m)
                {
                    m=d;
                    c=j;
                    r=i;
                }
            }
        }
    }
    return QPoint(c,r);
}

QPoint Iscissor::coordinateTransfer(QPoint matpoint){
    int c = matpoint.column;
    int r = matpoint.row;
    int newc = 320 - r*320.0/(grayImageMat.rows);
    int newr = c * 524.0/(grayImageMat.cols);
    return QPoint(newc,newr);
}

void Iscissor::deleteAllNodes()
{
    int h=pixelNodes.size();
    int w=pixelNodes[0].size();
    for(int i=0;i<h;i++)
    {
        for(int j=0;j<w;j++)
        {
            delete pixelNodes[i][j];
            pixelNodes[i][j]=NULL;
        }
        pixelNodes[i].clear();
    }
    pixelNodes.clear();
    seed=NULL;
}

Iscissor::~Iscissor()
{
    deleteAllNodes();
}

void Iscissor::setImage(Mat& image)
{
    deleteAllNodes();
    init_Image(image);
    costFunctionPaper();
}
void Iscissor::costFunctionModify()
{
    int height=grayImageMat.rows;
    int width=grayImageMat.cols;
    double maxD=-1.0;
    for(int j=0;j<height;j++)
        for(int i=0;i<width;i++)
        {
            PixelNode *pn=pixelNodes[j][i];
            for(int k=0;k<8;k++)
            {
                pn->setLinkCost(k,getD(i,j,k));
                if(pn->LinkCost(k)>maxD)
                    maxD=pn->LinkCost(k);
            }
        }
    for(int j=0;j<height;j++)
        for(int i=0;i<width;i++)
        {
            PixelNode *pn=pixelNodes[j][i];
            for(int k=0;k<8;k++)
            {
                double D=pn->LinkCost(k);
                double length=k%2==0?1.0:sqrt(2.0);
                pn->setLinkCost(k,(maxD-D)*length);
            }
        }
}

double Iscissor::getD(int i, int j, int link){
    double D = 0;
    double d;
    if(link==0)
    {
        if(j==0||j==grayImageMat.rows-1||i==grayImageMat.cols-1)
            return -1.0;
        int c0,c1,c2,c3;
        c0 = grayImageMat.at<int>(j-1,i);
        c1 = grayImageMat.at<int>(j-1,i+1);
        c2 = grayImageMat.at<int>(j+1,i);
        c3 = grayImageMat.at<int>(j+1,i+1);
        d=fabs((c0+c1)/2.0-(c2+c3)/2.0)/2.0;
    }
    else if(link==1)
    {
        if(j==0||i==grayImageMat.cols-1)
            return -1.0;
        int c0,c1;
        c0 = grayImageMat.at<int>(j,i+1);
        c1 = grayImageMat.at<int>(j-1,i);
        d=fabs(1.0 * (c0-c1))/sqrt(2.0);
    }
    else if(link==2)
    {
        if(i==0||j==0||i==grayImageMat.cols-1)
            return -1.0;
        int c0,c1,c2,c3;
        c0 = grayImageMat.at<int>(j,i-1);
        c1 = grayImageMat.at<int>(j-1,i-1);
        c2 = grayImageMat.at<int>(j,i+1);
        c3 = grayImageMat.at<int>(j-1,i+1);
        d=fabs((c0+c1)/2.0-(c2+c3)/2.0)/2.0;
    }
    else if(link==3)
    {
        if(i==0||j==0)
            return -1.0;
        int c0,c1;
        c0 = grayImageMat.at<int>(j-1,i-1);
        c1 = grayImageMat.at<int>(j,i-1);
        d=fabs(1.0 * (c0-c1))/sqrt(2.0);
    }
    else if(link==4)
    {
        int c0,c1,c2,c3;
        if(i==0||j==0||j==grayImageMat.rows-1)
            return -1.0;
        c0 = grayImageMat.at<int>(j-1,i);
        c1 = grayImageMat.at<int>(j-1,i-1);
        c2 = grayImageMat.at<int>(j+1,i-1);
        c3 = grayImageMat.at<int>(j+1,i);
        d=fabs((c0+c1)/2.0-(c2+c3)/2.0)/2.0;
    }
    else if(link==5)
    {
        if(i==0||j==grayImageMat.rows-1)
            return -1.0;
        int c0,c1;
        c0 = grayImageMat.at<int>(j,i-1);
        c1 = grayImageMat.at<int>(j+1,i);
        d=fabs(1.0 * (c0-c1))/sqrt(2.0);
    }
    else if(link==6)
    {
        int c0,c1,c2,c3;
        if(i==0||i==grayImageMat.cols-1||j==grayImageMat.rows-1)
            return -1.0;
        c0 = grayImageMat.at<int>(j,i-1);
        c1 = grayImageMat.at<int>(j+1,i-1);
        c2 = grayImageMat.at<int>(j+1,i+1);
        c3 = grayImageMat.at<int>(j,i+1);
        d=fabs((c0+c1)/2.0-(c2+c3)/2.0)/2.0;
    }
    else
    {
        int c0,c1;
        if(i==grayImageMat.cols-1||j==grayImageMat.rows-1)
            return -1.0;
        c0 = grayImageMat.at<int>(j,i+1);
        c1 = grayImageMat.at<int>(j+1,i);
        d=fabs(1.0* (c0-c1))/sqrt(2.0);
    }
    D = d;
    return D;
}

cv::Point Iscissor::unitGradVector(int c, int r){
    double dx=gradX.at<double>(r,c);
    double dy=gradY.at<double>(r,c);
    
    double mag = sqrt(dx*dx + dy*dy);
    mag = mag>1e-100?mag:1e-100;
    return cv::Point(dx/mag,dy/mag);
    
}


//what is the dist???
void Iscissor::computeSides(int dist){
    for (int i=0; i<gradX.rows; i++) {
        for (int j = 0; j<gradX.cols; j++) {
            cv::Point tmpUnitVetor = unitGradVector(j, i);
            int ix = round(j+dist*tmpUnitVetor.y);
            int iy = round(i-dist*tmpUnitVetor.x);
            int ox = round(j-dist*tmpUnitVetor.y);
            int oy = round(i+dist*tmpUnitVetor.x);
            
            ix = ix<gradX.cols-1?ix:gradX.cols-1;
            ix = ix>0?ix:0;
            ox = ox<gradX.cols-1?ox:gradX.cols-1;
            ox = ox>0?ox:0;
            iy = iy<gradX.rows-1?iy:gradX.rows;
            iy = iy>0?iy:0;
            oy = oy<gradX.rows-1?oy:gradX.rows;
            oy = oy>0?oy:0;
            insides.at<double>(i,j) = grayImageMat.at<double>(iy,ix);
            outsides.at<double>(i,j) = grayImageMat.at<double>(oy,ox);
        }
    }
}

int Iscissor::getTrainingIdx(int granularity,double value){
    return round(value*(granularity-1));
}

void Iscissor::gaussianBlur(vector<double> buffer,vector<double> output){
    
}

void Iscissor::calculateTraining(vector<QPoint> trainingPoints,int granularity,Mat input, vector<double> output){
    vector<double> buffer;
    buffer.resize(granularity);
    for (int i= 0; i<granularity; i++) {
        buffer[i] = 0;
    }
    int maxVal = 1;
    for (int i =0;i<trainingPoints.size(); i++) {
        QPoint tmpTrainingPoint = trainingPoints[i];
        int idx = getTrainingIdx(granularity, input.at<double>(tmpTrainingPoint.row,tmpTrainingPoint.column));
        buffer[idx] += 1;
        maxVal = maxVal>buffer[idx]?maxVal:buffer[idx];
    }
    for (int i=0; i<granularity; i++) {
        buffer[i] = 1- buffer[i]/maxVal;
    }
    GaussianBlur(buffer, output, cv::Size(3,3),0.0);
}

void Iscissor::doTraing(vector<QPoint> points){
    if (points.size()<8) {
        return;
    }
    normalize(grayImageMat, normalizedGrayImage, 0,1, NORM_MINMAX);
    calculateTraining(points, 256, normalizedGrayImage, edgeTraining);
    calculateTraining(points, 1024,gradXY , gradTraining);
    calculateTraining(points, 256, insides, insideTraining);
    calculateTraining(points, 256, outsides, outsideTraining);
}

double Iscissor::getTrainedEdge(double edge){
    return edgeTraining[getTrainingIdx(256, edge)];
}

double Iscissor::getTrainedGrad(double grad){
    return gradTraining[getTrainingIdx(1024, grad)];
}

double Iscissor::getTrainedInside(double inside){
    return insideTraining[getTrainingIdx(256, inside)];
}

double Iscissor::getTrainedOutside(double outside){
    return outsideTraining[getTrainingIdx(256, outside)];
}

double Iscissor::dist(PixelNode *ori, PixelNode *neighbor){
    double grad = gradXY.at<double>((int)neighbor->Row(),(int)neighbor->Column());
    double lap = LapBeforeScale.at<double>((int)neighbor->Row(),(int)neighbor->Column());
    double dir = gradDirection(ori, neighbor);
    double gradT = getTrainedGrad(grad);
    double edgeT = getTrainedEdge(normalizedGrayImage.at<double>((int)ori->Row(),(int)ori->Column()));
    double insideT = getTrainedInside(insides.at<double>((int)ori->Row(),(int)ori->Column()));
    double outsideT = getTrainedOutside(outsides.at<double>((int)ori->Row(),(int)ori->Column()));
    return 0.3*gradT + 0.3*lap + 0.1*(dir + edgeT + insideT + outsideT);
}

double Iscissor::gradDirection(PixelNode *ori, PixelNode *nei){
    cv::Point oriUnit = unitGradVector((int)ori->Column(), (int)ori->Row());
    cv::Point neiUnit = unitGradVector((int)nei->Column(), (int)nei->Row());
    double oriTmp = oriUnit.y * ((int)nei->Column()-(int)ori->Column()) - oriUnit.x * ((int)nei->Row()-(int)ori->Row());
    double neiTmp = neiUnit.y * ((int)nei->Column()-(int)ori->Column()) - neiUnit.x * ((int)nei->Row()-(int)ori->Row());
    if (oriTmp<0) {
        oriTmp = -oriTmp;
        neiTmp = -neiTmp;
    }
    if (ori->Column()!=nei->Column() && ori->Row()!=nei->Row()) {
        oriTmp *= 0.707;
        neiTmp *= 0.707;
    }
    return 2.09 * (acos(neiTmp)+ acos(oriTmp));
}


void Iscissor::costFunctionPaper(){
    Mat lap(grayImageMat.rows,grayImageMat.cols,CV_64F);
    Laplacian(grayImageMat,lap,CV_64F);
    LapBeforeScale = lap;
    //visualCVMat(lap,"lap");
    Mat sobx(grayImageMat.rows,grayImageMat.cols,CV_64F);
    Mat soby(grayImageMat.rows,grayImageMat.cols,CV_64F);
    Sobel(grayImageMat,sobx,CV_64F,1,0);
    Sobel(grayImageMat,soby,CV_64F,0,1);
    gradX = sobx;
    gradY = soby;
    //visualCVMat(sobx,"sobx");
    //visualCVMat(soby,"soby");
    Mat grad(grayImageMat.rows,grayImageMat.cols,CV_64F);
    Mat fz(grayImageMat.rows,grayImageMat.cols,CV_64F);
    Mat fg(grayImageMat.rows,grayImageMat.cols,CV_64F);
    Mat Dx(grayImageMat.rows,grayImageMat.cols,CV_64F);
    Mat Dy(grayImageMat.rows,grayImageMat.cols,CV_64F);
    double maxG = 0;
    for (int i =0; i<grayImageMat.rows; i++) {
        for (int j=0; j<grayImageMat.cols; j++) {
            double dx=sobx.at<double>(i,j);
            double dy=soby.at<double>(i,j);
            double g=sqrt(dx*dx+dy*dy);
            grad.at<double>(i,j)=g;
            if(maxG<g)
                maxG=g;
            if(abs(g)>0.0001)
            {
                Dx.at<double>(i,j)=dy/g;
                Dy.at<double>(i,j)=-dx/g;
            }
            else
            {
                Dx.at<double>(i,j)=0;
                Dy.at<double>(i,j)=0;
            }
        }
    }
    
    for(int i=0;i<grayImageMat.rows;i++)
        for(int j=0;j<grayImageMat.cols;j++)
        {
            fz.at<double>(i,j)=abs(lap.at<double>(i,j))<0.0001?0:1;
            fg.at<double>(i,j)=1.0-grad.at<double>(i,j)/maxG;
            
        }
    gradXY = fg;
    double wz=0.43,wd=0.43,wg=0.14;
    double mmax=DBL_MIN,mmin=DBL_MAX;
    for(int i=0;i<grayImageMat.rows;i++)
        for(int j=0;j<grayImageMat.cols;j++)
        {
            PixelNode *pn=pixelNodes[i][j];
            double fd;
            for(int k=0;k<8;k++)
            {
                int c,r;
                pn->Neighbor(k,c,r);
                if(c>=0&&c<grayImageMat.cols&&r>=0&&r<grayImageMat.rows)
                {
                    Vec2d l(c-j,r-i);
                    Vec2d Dp(Dx.at<double>(i,j),Dy.at<double>(i,j));
                    Vec2d Dq(Dx.at<double>(r,c),Dy.at<double>(r,c));
                    if(k%2)
                        l=l/sqrt(2.0);
                    double dp=l.dot(Dp);
                    if(dp<0)
                    {
                        l=-l;
                        dp=-dp;
                    }
                    double dq=l.dot(Dq);
                    double fd=(acos(dp)+acos(dq))/CV_PI;
                    double v;
                    if(k%2)
                        v=wz*fz.at<double>(r,c)+wg*fg.at<double>(r,c)+wd*fd;
                    else
                        v=wz*fz.at<double>(r,c)+wg*fg.at<double>(r,c)/sqrt(2.0)+wd*fd;
                    if(v>mmax)
                        mmax=v;
                    if(v<mmin)
                        mmin=v;
                    pn->setLinkCost(k,v);
                }
            }
        }
    for(int i=0;i<grayImageMat.rows;i++)
        for(int j=0;j<grayImageMat.cols;j++)
        {
            PixelNode *pn=pixelNodes[i][j];
            for(int k=0;k<8;k++)
                pn->setLinkCost(k,(pn->LinkCost(k)-mmin)/(mmax-mmin)*170.0);
        }
    std::cout<<"Cost Function Completed";
    
}



