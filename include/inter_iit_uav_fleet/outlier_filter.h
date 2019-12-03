#ifndef INTER_IIT_UAV_FLEET_OUTLIER_FILTER_H
#define INTER_IIT_UAV_FLEET_OUTLIER_FILTER_H

#include <opencv2/opencv.hpp>
// #include <opencv-3.3.1-dev/opencv2/opencv.hpp>

double crossProduct(cv::Point, cv::Point, cv::Point);
double baseLength(cv::Point, cv::Point);
double pointToLineDistance(const std::vector<cv::Point>&, int, int);

double crossProduct(cv::Point point, cv::Point line_start, cv::Point line_end){
   return fabs((point.x - line_start.x) * (line_end.y - line_start.y) - (point.y - line_start.y) * (line_end.x - line_start.x)) ;   
}

double baseLength(cv::Point line_end, cv::Point line_start){
    return cv::norm(line_end - line_start);
}

double pointToLineDistance(const std::vector<cv::Point>& contour, int i, int n){
        if(i-n>=0 && i+n<contour.size())
            return (crossProduct(contour.at(i),contour.at(i-n),contour.at(i+n))/baseLength(contour.at(i+n),contour.at(i-n)));
        else if(i-n<0 && i+n<contour.size())
            return (crossProduct(contour.at(i),contour.at(i-n + contour.size()),contour.at(i+n))/baseLength(contour.at(i+n),contour.at(i-n + contour.size())));
        else if(i-n>=0 && i+n>=contour.size())
            return (crossProduct(contour.at(i),contour.at(i-n),contour.at(i+n-contour.size()))/baseLength(contour.at(i+n-contour.size()),contour.at(i-n)));
}

void outlier_filter(const std::vector<cv::Point>& contour, const std::vector<int>& hull, std::vector<cv::Point>& corners)
{
    int i,j;
    int size = hull.size();
    int perimeter = contour.size();
    int delta = (round) (perimeter/8.0);

    std::vector<double> distances(contour.size(), 0);
    std::vector<double> signature(contour.size(), 0);

    for(i=0;i<size;i++)
        distances.at(hull.at(i)) = pointToLineDistance(contour, hull.at(i), delta);

    if(distances.at(0)>=distances.at(1) && distances.at(0)>=distances.at(contour.size()-1) && distances.at(0)>=0.02*contour.size())
        signature.at(0)=distances.at(0);

    if(distances.at(contour.size()-1)>=distances.at(contour.size()-2) && distances.at(contour.size()-1)>=distances.at(0) && distances.at(contour.size()-1)>=0.02*contour.size())
        signature.at(contour.size()-1)=distances.at(contour.size()-1);

    for(i=1;i<contour.size()-1;i++)
    {
        if(distances.at(i)>=distances.at(i+1) && distances.at(i)>=distances.at(i-1) && distances.at(i)>=0.02*contour.size())
            signature.at(i)=distances.at(i);
        else
            signature.at(i)=0;
    }
    int x0=0, y0=0;

    for(i = 0;i<delta;i++)
    {
        if(signature.at(i)==0)
            continue;
        else
        {
            x0 = 0;
            y0 = 0;
            for(int j=contour.size()-delta+i;j<contour.size();j++)//Handling cases at the end of the vector
            {
                if(signature.at(j)==0)
                    continue;
                else
                {
                    if(signature.at(j)>y0)
                    {
                        x0=j;
                        y0=signature.at(j);
                    }
                    signature.at(j)=0;
                }
            }
            for(int j=0;j<=delta+i;j++)//Handling cases at the start of the vector
            {
                if(signature.at(j)==0)
                    continue;
                else
                {
                    if(signature.at(j)>y0)
                    {
                        x0=j;
                        y0=signature.at(j);
                    }
                    signature.at(j)=0;
                }
            }
            signature.at(x0)=y0;
        }
    }

    for(i = delta;i<contour.size()-delta;i++)
    {
        if(signature.at(i)==0)
            continue;
        else
        {
            x0 = 0;
            y0 = 0;
            for(int j=i-delta;j<=i+delta;j++)
            {
                if(signature.at(j)==0)
                    continue;
                else
                {
                    if(signature.at(j)>y0)
                    {
                        x0=j;
                        y0=signature.at(j);
                    }
                    signature.at(j)=0;
                }
            }
            signature.at(x0)=y0;
        }
    }

    for(i = contour.size()-delta;i<contour.size();i++)
    {
        if(signature.at(i)==0)
            continue;
        else
        {
            x0 = 0;
            y0 = 0;
            for(int j=i-delta;j<contour.size();j++)//Handling cases at the end of the vector
            {
                if(signature.at(j)==0)
                    continue;
                else
                {
                    if(signature.at(j)>y0)
                    {
                        x0=j;
                        y0=signature.at(j);
                    }
                    signature.at(j)=0;
                }
            }
            for(int j=0;j<=delta+i-contour.size();j++)//Handling cases at the start of the vector
            {
                if(signature.at(j)==0)
                    continue;
                else
                {
                    if(signature.at(j)>y0)
                    {
                        x0=j;
                        y0=signature.at(j);
                    }
                    signature.at(j)=0;
                }
            }
            signature.at(x0)=y0;
        }
    }

    for(i=0;i<signature.size();i++)
    {
        if(signature.at(i)!=0)
            corners.push_back(contour.at(i));
    }
}

#endif