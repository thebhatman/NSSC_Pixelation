#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
 
using namespace cv;
using namespace std;

Mat binary(Mat a)
{
    int i,j;
    for(i=0; i<a.rows; i++)
        for(j=0; j<a.cols; j++)
        {
            if(a.at<uchar>(i,j) < 50) a.at<uchar>(i,j) = 0;
            else a.at<uchar>(i,j) = 255;
        }

    return a;
}

int isValid(int i, int j, int rows, int cols)
{
	if(i < 0 || j < 0 || i >= rows || j >= cols)
	{
		return 0;
	}
	return 1;
}

Mat isolate_path(Mat c, Mat path)
{
	int i,j,k, flag = 0;
	k = 5;
	for(i = 0; i < c.rows; i++)
	{
		for(j = 0; j < c.cols;j++)
		{
			flag = 0;
			if(isValid(i + k,j,c.rows,c.cols) && isValid(i, j + k, c.rows, c.cols))
			{
				if(path.at<uchar>(i+k,j) >= 128 && path.at<uchar>(i-k, j) >= 128 && c.at<uchar>(i,j) < 100)
				{
					//cout<<"alright"<<endl;
					flag = 1;
				}
				if(path.at<uchar>(i,j+k) >= 128 && path.at<uchar>(i, j-k) >= 128 && c.at<uchar>(i,j) < 100)
				{
					//cout<<"alright"<<endl;
					flag = 1;
				}
				if(path.at<uchar>(i+k,j+k) >= 128 && path.at<uchar>(i-k, j-k) >= 128 && path.at<uchar>(i-k,j+k) >= 128 && path.at<uchar>(i+k,j-k) >=128 && c.at<uchar>(i,j) < 100)
				{
					//cout<<"alright"<<endl;
					flag = 1;
				}
			}
			if(!flag)
			{
				c.at<uchar>(i,j) = 255;
			}
		}
	}
	return c;
}


int Gx(int i, int j, Mat a)
{
	int p,q;

	int x[3][3]={{-3,0,3},{-10,0,10},{-3,0,3}};

	int gx=0;

	for(p=i-1;p<=i+1;p++)
		for(q=j-1;q<=j+1;q++)
				gx+=a.at<uchar>(p,q)*x[p-i+1][q-j+1];

	return abs(gx)/16;

}

int Gy(int i, int j, Mat a)
{
	int p,q;

	int y[3][3]={{3,10,3},{0,0,0},{-3,-10,-3}};

	int gy=0;

	for(p=i-1;p<=i+1;p++)
		for(q=j-1;q<=j+1;q++)
				gy+=a.at<uchar>(p,q)*y[p-i+1][q-j+1];

	return abs(gy)/4;

}

typedef struct point{
	int i,j;
	vector<Point2d> neighbours;
}point;


int main()
{
	Mat a = imread("pxl.png", 0);
	int i,j,low=130,high=20;

	namedWindow("Image",WINDOW_AUTOSIZE);
	//createTrackbar("Low","Image",&low,500);
	//createTrackbar("High","Image",&high,500);
	Mat b(a.rows,a.cols,CV_8UC1,Scalar(0));

	while(1)
	{
		GaussianBlur(a,b,Size(5,5),1.5,1.5);
		Canny(b,b,low,high);

		imshow("Image",b);
		int flag=waitKey(30);
		if(flag==27) break;
	}
	vector<std::vector<cv::Point> > contours;
    findContours(b, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    Mat path = Mat::zeros(b.size(), CV_8UC1);
    drawContours(path, contours, -1, CV_RGB(255,255,255), CV_FILLED);

    /*namedWindow("Contours",WINDOW_NORMAL);
    imshow("Contours",path);
    while(1)
    {
        int flag = waitKey(10);
        if(flag == 27) break;
    }*/
    Mat kernel = Mat::ones(9,9, CV_8UC1); 
    dilate(path, path, kernel);

    /*imshow("Dilated",path);
    while(1)
    {
        int flag = waitKey(10);
        if(flag == 27) break;
    } */
    Mat c = path.clone();
    c = binary(c);
    c = isolate_path(c, path);
    //cout<<"I am out"<<endl;
    namedWindow("isolated_path", WINDOW_NORMAL);
    imshow("isolated_path",c);
    while(1)
    {
        int flag = waitKey(10);
        if(flag == 27) break;
    } 

    Mat d(c.rows, c.cols, CV_8UC1, Scalar(255));
    int x_grad, y_grad;
    for(i = 0; i < c.rows; i++)
    {
    	for(j = 0; j < c.cols; j++)
    	{
    		if(Gx(i,j,c) > 180 && Gy(i,j,c) > 80)
    		{
    			d.at<uchar>(i,j) = 0;
    		}
    	}
    }

    namedWindow("corners", WINDOW_NORMAL);
    imshow("corners", d);
    while(1)
    {
        int flag = waitKey(10);
        if(flag == 27) break;
    } 
    vector<point> corners;
    point temp;
    for(i = 0; i < d.rows; i++)
    {
   		for(j = 0; j < d.cols; j++)
   		{
   			if(d.at<uchar>(i,j) == 0)
   			{
   				temp.i = i; temp.j = j;
   				corners.push_back(temp);
   				//cout<<temp.i<<" "<<temp.j<<endl;
   			}
   		}
    } 
    Point2d closer;
    for(i = 0; i < corners.size(); i++)
    {
    	for(j = i + 1; j < corners.size(); j++)
    	{
    		if(sqrt(pow(corners[i].i - corners[j].i,2) + pow(corners[i].j - corners[j].j,2)) < 30)
    		{
    			closer.x = corners[j].i;
    			closer.y = corners[j].j;
    			corners[i].neighbours.push_back(closer);
    			closer.x = corners[i].i;
    			closer.y = corners[i].j;
    			corners[j].neighbours.push_back(closer);
    		}
    	}
    }
    vector<point> averaged_corners;
    point middle_point;

    for(i = 0; i < corners.size(); i++)
    {
    	cout<<"For corner "<<corners[i].i<<" "<<corners[i].j<<":"<<endl;
    	middle_point.i = corners[i].i; middle_point.j = corners[i].j;
    	for(j = 0; j < corners[i].neighbours.size(); j++)
    	{
    		//cout<<corners[i].neighbours[j].x<<" "<<corners[i].neighbours[j].y<<endl;
    		middle_point.i += corners[i].neighbours[j].x;
    		middle_point.j += corners[i].neighbours[j].y;
    	}
    	middle_point.i /= (1 + corners[i].neighbours.size());
    	middle_point.j /= (1 + corners[i].neighbours.size());
    	cout<<"mean corner point: "<<middle_point.i<<" "<<middle_point.j<<endl;
    	averaged_corners.push_back(middle_point);
    }
    Mat e(d.rows, d.cols, CV_8UC1, Scalar(255));
    for(i = 0; i < averaged_corners.size(); i++)
    {
    	e.at<uchar>(averaged_corners[i].i, averaged_corners[i].j) = 0;
    }
    namedWindow("averaged_corners", WINDOW_NORMAL);
    imshow("averaged_corners", e);
    while(1)
    {
        int flag = waitKey(10);
        if(flag == 27) break;
    } 
	return 0;
}