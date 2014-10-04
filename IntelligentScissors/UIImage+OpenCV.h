//
//  UIImage+OpenCV.h
//  DoctorAssitant
//
//  Created by Chao Li on 8/6/14.
//  Copyright (c) 2014 Luis Alvarado. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface UIImage (OpenCV)

#pragma mark Generate UIImage from cv::Mat
+ (UIImage*)fromCVMat:(const cv::Mat&)cvMat;

#pragma mark Generate cv::Mat from UIImage
+ (cv::Mat)toCVMat:(UIImage *)image;
- (cv::Mat)toCVMat;

@end
