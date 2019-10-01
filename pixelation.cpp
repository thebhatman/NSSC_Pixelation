#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <math.h>
 
using namespace cv;
using namespace std;

void sendCommand(char command) {            // serial communication
    char send = command;
    FILE *serport = fopen("/dev/ttyACM0","w");

    if(serport!=NULL) {
        fprintf(serport,"%c\n",send);
        fclose(serport);
        printf("%c\n",send);
    }
    else {
        printf("port not open");
    }
}

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

float dist_two_points(point p1, point p2)
{
	return sqrt(pow(p1.i-p2.i,2) + pow(p1.j - p2.j,2));
}

int does_path_exist(point p1, point p2, Mat c)
{
	int i,j=0,k;
	int distance = sqrt(pow(p1.i - p2.i, 2) + pow(p1.j - p2.j, 2));
	point in_between;
	for(k = 0; k <= distance; k++)
	{
		in_between.i = (k*p2.i + (distance - k)*p1.i)/distance;
		in_between.j = (k*p2.j + (distance - k)*p1.j)/distance;
		if(c.at<uchar>(in_between.i, in_between.j) > 128)
		{
			j++;
		}
	}
	if(j > 5)      //Approximation but works like magic
		return 0;
	return 1;
}
vector<float> dist(50);
vector<int> baap_ka_index(50);

int index_of_best_node(int *visited, int** path_matrix, vector<point> &distinct_corners)
{
	int i,j,k;
	int n = distinct_corners.size();
	int  min_dist = 10000000;
	j = -1;
	for(i = 0; i < n; i++)
	{
		if(dist[i] < min_dist && visited[i] == 0)
		{
			min_dist = dist[i];
			j = i;
		}
	}
	return j;
}

void dijkstra(vector<point> &distinct_corners, int start_index, int end_index, int *visited, int** path_matrix, Mat a)
{
	int i,j,k;
	int n = distinct_corners.size();
	for(i = 0; i < n; i++)
	{
		if(i != start_index)
		{
			dist[i] = 10000000;
		}
		else
			dist[i] = 0;
		baap_ka_index[i] = -1;
	}
	int curr_node_index;
	float temp_dist;
	while(1)
	{
		cout<<"Dijkstra running..."<<endl;
		if(curr_node_index == end_index) break; 
		curr_node_index = index_of_best_node(visited, path_matrix, distinct_corners);
		//if(visited[curr_node_index] == 1) continue;
		cout<<"curr_node_index = "<<curr_node_index<<endl;
		cout<<"curr_node "<<distinct_corners[curr_node_index].i<<" "<<distinct_corners[curr_node_index].j<<endl;
		//cout<<path_matrix[curr_node_index][0]<<endl;
		for(j = 0; j < n; j++)
		{
			if(curr_node_index == -1) continue;
			if(path_matrix[curr_node_index][j] == 1)
			{
				temp_dist = dist[curr_node_index] + dist_two_points(distinct_corners[curr_node_index], distinct_corners[j]);
				if(temp_dist < dist[j])
				{
					dist[j] = temp_dist;
					baap_ka_index[j] = curr_node_index;
				}
			}
		}
		//cout<<baap_ka_index[j]<<endl;
		visited[curr_node_index] = 1;
	}
	cout<<"Hello"<<endl;   //Just like that :P

}



void draw_final_path(vector<point> &distinct_corners, int end_index, int start_index, Mat b)
{
	Point2d curr_node;
	curr_node.x = distinct_corners[end_index].j;
	curr_node.y = distinct_corners[end_index].i;
	Point2d baap_node;
	int curr_index = end_index;
	int baap_index = baap_ka_index[curr_index];
	while(1)
	{
		if(curr_node.x == distinct_corners[start_index].j && curr_node.y == distinct_corners[start_index].i)
		{
			break;
		}
		baap_node.x = distinct_corners[baap_index].j;
		baap_node.y = distinct_corners[baap_index].i;
		line(b, curr_node, baap_node, Scalar(255,0,0), 2 , 8);
		curr_node = baap_node;
		curr_index = baap_index;
		baap_index = baap_ka_index[curr_index];
		
	}
}

void jas(void)
{
	cout<<"this is the test function-Ari";
}

int main()
{
	Mat a = imread("pxl.png", 0);
	int i,j,low=130,high=20;
    char pizza;

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
    Mat temp_path = path.clone();
    Mat temp_kernel = Mat::ones(4,4, CV_8UC1);
    dilate(temp_path, temp_path, temp_kernel);
    Mat kernel = Mat::ones(9,9, CV_8UC1); 
    dilate(path, path, kernel);

    imshow("Dilated",temp_path);
    while(1)
    {
        int flag = waitKey(10);
        if(flag == 27) break;
    } 
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
    	middle_point.i = corners[i].i; middle_point.j = corners[i].j;
    	for(j = 0; j < corners[i].neighbours.size(); j++)
    	{
    		middle_point.i += corners[i].neighbours[j].x;
    		middle_point.j += corners[i].neighbours[j].y;
    	}
    	middle_point.i /= (1 + corners[i].neighbours.size());
    	middle_point.j /= (1 + corners[i].neighbours.size());
    	averaged_corners.push_back(middle_point);
    }
    Mat e(d.rows, d.cols, CV_8UC1, Scalar(255));
    for(i = 0; i < averaged_corners.size(); i++)
    {
    	e.at<uchar>(averaged_corners[i].i, averaged_corners[i].j) = 0;
    }
    vector<point> distinct_corners;
    for(i = 0; i < averaged_corners.size(); i++)
    {
    	for(j = 0; j < i; j++)
    	{
    		if(averaged_corners[i].i == averaged_corners[j].i && averaged_corners[i].j == averaged_corners[j].j)
    		{
    			break;
    		}
    	}
    	if(i == j)
    		distinct_corners.push_back(averaged_corners[i]);
    }
    /*for(i = 0; i < distinct_corners.size(); i++)
    {
    	cout<<i<<endl;
    	cout<<distinct_corners[i].i<<" "<<distinct_corners[i].j<<endl;
    }*/
    namedWindow("averaged_corners", WINDOW_NORMAL);
    imshow("averaged_corners", e);
    while(1)
    {
        int flag = waitKey(10);
        if(flag == 27) break;
    } 
    point start_node, end_node;
    start_node.i = distinct_corners[24].i;  //These indices are hardcoded. Might need to change them on the D-Day.
    start_node.j = distinct_corners[24].j;
    end_node.i = distinct_corners[5].i;
    end_node.j = distinct_corners[5].j;
    int **path_matrix = new int*[distinct_corners.size()];
    for(i = 0; i < distinct_corners.size(); i++)
    {
    	path_matrix[i] = new int[distinct_corners.size()];
    }
    for(i = 0; i < distinct_corners.size(); i++)
    {
    	path_matrix[i][i] = 0;
    	for(j = 0; j < distinct_corners.size(); j++)
    	{
    		if(i != j)
    		{
    			path_matrix[i][j] = does_path_exist(distinct_corners[i], distinct_corners[j], temp_path);
    			cout<<distinct_corners[i].i<<" "<<distinct_corners[i].j<<" ---> "<<distinct_corners[j].i<<" "<<distinct_corners[j].j<<" "<<path_matrix[i][j]<<endl;
			} 
    	}
    }
    int *visited = new int[distinct_corners.size()];
    for(i = 0; i < distinct_corners.size(); i++)
    	visited[i] = 0;
    dijkstra(distinct_corners, 24, 5, visited, path_matrix, b);
    for(i = 0; i < distinct_corners.size(); i++)
    {
    	cout<<"baccha: "<<distinct_corners[i].i<<" "<<distinct_corners[i].j<<endl;
    	cout<<"baap: "<<distinct_corners[baap_ka_index[i]].i<<" "<<distinct_corners[baap_ka_index[i]].j<<endl;
    }
    draw_final_path(distinct_corners, 5, 24, a);
    namedWindow("Final Path", WINDOW_NORMAL);
    imshow("Final Path", a);
    while(1)
    {
    	int flag = waitKey(10);
        if(flag == 27) break;
    }
	return 0;
}
