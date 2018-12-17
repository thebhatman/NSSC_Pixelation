#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <limits>
 
using namespace cv;
using namespace std;

#define COL1 221
#define COL2 989
#define ROW1 10
#define ROW2 713
#define PI 3.14159265

int thresh = 125;
int max_thresh = 255;

float dist_two_points(Point2d p1, Point2d p2)
{
	return sqrt(pow(p1.x-p2.x,2) + pow(p1.y - p2.y,2));
}

int isyellow(Mat a, int i, int j)
{
	if(a.at<Vec3b>(i,j)[2] > 245 && a.at<Vec3b>(i,j)[1] > 190 && a.at<Vec3b>(i,j)[1] < 220 && a.at<Vec3b>(i,j)[0] > 100 &&
		a.at<Vec3b>(i,j)[0] < 135) return 1;
	return 0;
}

int isblue(Mat a, int i, int j)
{
	if(a.at<Vec3b>(i,j)[0] > 230 && a.at<Vec3b>(i,j)[1] > 165 && a.at<Vec3b>(i,j)[1] < 190 &&
		a.at<Vec3b>(i,j)[2] < 35) return 1;
	return 0;
}

int isblack(Mat a, int i, int j)
{
	if(a.at<Vec3b>(i,j)[0] < 150 && a.at<Vec3b>(i,j)[1] < 150 && a.at<Vec3b>(i,j)[2] < 150 &&
		abs(a.at<Vec3b>(i,j)[0] - a.at<Vec3b>(i,j)[1]) < 15 && abs(a.at<Vec3b>(i,j)[1] - a.at<Vec3b>(i,j)[2]) < 15 &&
		abs(a.at<Vec3b>(i,j)[2] - a.at<Vec3b>(i,j)[0]) < 15) return 1;
	return 0;
}

Mat binary(Mat a)
{
    int i,j;
    for(i=0; i<a.rows; i++)
        for(j=0; j<a.cols; j++)
        {
            if(a.at<uchar>(i,j)<125) a.at<uchar>(i,j) = 0;
            else a.at<uchar>(i,j) = 255;
        }

    return a;
}

int isgreen(Mat a, int i, int j)
{ 
	if(a.at<Vec3b>(i,j)[1] - a.at<Vec3b>(i,j)[2] > 20 && a.at<Vec3b>(i,j)[1] - a.at<Vec3b>(i,j)[0] > 20) return 1;
	return 0;
}

int isred(Mat a, int i, int j)
{
	if(a.at<Vec3b>(i,j)[2] - a.at<Vec3b>(i,j)[0] > 50 && a.at<Vec3b>(i,j)[2] - a.at<Vec3b>(i,j)[1] > 50) return 1;
	return 0;
}

void find_src_dest(Mat a, Point2d array[])
{
	int i1,i2,j;
	int Gflag = 1, Rflag = 1;
	for(i1=0; i1<a.rows && (Gflag || Rflag); i1++)
		for(j=0 ;j<a.cols && (Gflag || Rflag); j++)
		{
			if(isgreen(a,i1,j) && Gflag)
			{
				for(i2=i1+1; isgreen(a,i2,j); i2++);
				i2--;
				array[0].x = (i1+i2)>>1;
				array[0].y = j;
				Gflag = 0;
			}

			if(isred(a,i1,j) && Rflag)
			{
				for(i2=i1+1; isred(a,i2,j); i2++);
				i2--;
				array[1].x = (i1+i2)>>1;
				array[1].y = j;
				Rflag = 0;	
			}
		}
}

Mat remove_src_dest(Mat a)
{
	int i,j,k;
	for(i=0; i<a.rows; i++)
		for(j=0; j<a.cols; j++)
			if(isgreen(a,i,j) || isred(a,i,j))
				for(k=0; k<=2; k++) a.at<Vec3b>(i,j)[k] = 0;

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
	k = 20;
	for(i = 0; i < c.rows; i++)
	{
		for(j = 0; j < c.cols;j++)
		{
			flag = 0;
			if(isValid(i + k,j,c.rows,c.cols) && isValid(i, j + k, c.rows, c.cols) && isValid(i-k,j,c.rows,c.cols) && isValid(i,j-k,c.rows,c.cols))
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

Mat cornerHarris_demo(Mat c)
{

  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( c.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  cornerHarris( c, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  Mat corners(c.rows, c.cols, CV_8UC1, Scalar(0));
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
               corners.at<uchar>(j,i) = 255;
              }
          }
     }
  /// Showing the result
  /*namedWindow( "cornerHarris", WINDOW_NORMAL);
  namedWindow( "corners", WINDOW_NORMAL);
  imshow( "cornerHarris", dst_norm_scaled );
  imshow( "corners", corners );
  while(1)
  {
  	int flag = waitKey(10);
  	if(flag == 27) break;
  }*/

  return corners;
}


vector<Point2d> refine_corners(Mat corners, int k)
{
	vector<Point2d> vertices;
	int i,j;
	for(i=0; i<corners.rows; i++)
		for(j=0; j<corners.cols; j++)
		{
			if(corners.at<uchar>(i,j) == 255)
			{	
				vector<Point2d> neighbors;
				int p,q;
				for(p=i-k; p<=i+k; p++)
					for(q=j-k; q<=j+k; q++)
					{	
						if(isValid(p,q,corners.rows,corners.cols) && corners.at<uchar>(p,q))
						{
							Point2d temp;
							temp.x = p;
							temp.y = q;
							neighbors.push_back(temp);
							corners.at<uchar>(p,q) = 0;
						}
					}
				//printf("No. of neighbors = %d\n",(int)neighbors.size());
				if(neighbors.size() == 0) continue;
				int x=0,y=0,iter;
				for(iter=0; iter<neighbors.size(); iter++)
				{
					x += neighbors[iter].x;
					y += neighbors[iter].y;
				}
				x /= neighbors.size();
				y /= neighbors.size();
				Point2d temp;
				temp.x = x;
				temp.y = y;
				vertices.push_back(temp);
			}
		}

	return vertices;
}

void path_exists(Mat a, vector<Point2d> &vertices, int **path_matrix)
{
	int i,j,k;
	for(i=0; i<vertices.size(); i++)
		for(j=i+1; j<vertices.size(); j++)
		{	Point2d p1 = vertices[i], p2 = vertices[j];
			float distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
			Point2d in_between;
			int flag = 0;
			for(k = 0; k <= distance; k++)
			{
				in_between.x = (k*p2.x + (distance - k)*p1.x)/distance;
				in_between.y = (k*p2.y + (distance - k)*p1.y)/distance;
				if(a.at<uchar>(in_between.x, in_between.y) == 255) {flag = 1; break;}
			}
			if(flag == 0){printf("(%f,%f) and (%f,%f) connected\n",vertices[i].x,vertices[i].y,
				vertices[j].x,vertices[j].y); path_matrix[i][j] = 1; path_matrix[j][i] = 1;}
		}
}

int index_of_best_node(int *visited, int** path_matrix, vector<Point2d> &distinct_corners, vector<float> &dist)
{
	int i,j,k;
	int n = distinct_corners.size();
	int  min_dist = INT_MAX;
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

void dijkstra(vector<Point2d> &distinct_corners, int start_index, int end_index, int *visited, int** path_matrix, vector<int> &baap_ka_index)
{
	int i,j,k;
	int n = distinct_corners.size();
	vector<float> dist(n);
	for(i = 0; i < n; i++)
	{
		if(i != start_index)
		{
			dist[i] = INT_MAX;
		}
		else
			dist[i] = 0;
		baap_ka_index[i] = -1;
	}
	int curr_node_index;
	float temp_dist;
	while(1)
	{
		//cout<<"Dijkstra running..."<<endl;
		if(curr_node_index == end_index) break; 
		curr_node_index = index_of_best_node(visited, path_matrix, distinct_corners, dist);
		//if(visited[curr_node_index] == 1) continue;
		cout<<"curr_node_index = "<<curr_node_index<<endl;
		cout<<"curr_node "<<distinct_corners[curr_node_index].x<<" "<<distinct_corners[curr_node_index].y<<endl;
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

void draw_final_path(vector<Point2d> &distinct_corners, int end_index, int start_index, Mat b, vector<int> &baap_ka_index, vector<Point2d> &graph)
{
	Point2d curr_node;
	curr_node.x = distinct_corners[end_index].x;
	curr_node.y = distinct_corners[end_index].y;
	graph.push_back(curr_node);
	Point2d baap_node;
	int curr_index = end_index;
	int baap_index = baap_ka_index[curr_index];
	while(1)
	{
		if(curr_node.x == distinct_corners[start_index].x && curr_node.y == distinct_corners[start_index].y)
		{
			break;
		}
		baap_node.x = distinct_corners[baap_index].x;
		baap_node.y = distinct_corners[baap_index].y;
		Point2d start, end;
		start.x = curr_node.y;
		start.y = curr_node.x;
		end.x = baap_node.y;
		end.y = baap_node.x;
		line(b, start, end, Scalar(255,0,0), 2 , 8);
		curr_node = baap_node;
		curr_index = baap_index;
		baap_index = baap_ka_index[curr_index];
		graph.push_back(curr_node);		
	}
}

void sendCommand(char send)            // serial communication
{    
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

void run_bot(vector<Point2d> graph)
{	
	VideoCapture cap(1);
	if(!cap.isOpened()) {printf("Video feed unavailable\n"); return;}
	//cap.set(CV_CAP_PROP_FRAME_WIDTH, )
	Mat a;
	int i,j,node=0;
	float threshold = 2;
	Point2d aim = graph[node];
	while(1)
	{
		cap >> a;
		
		vector<Point2d> yellow_points, blue_points;
		for(i=0; i<a.rows; i++)
			for(j=0; j<a.cols; j++)
			{
				if(isyellow(a,i,j))
				{
					Point2d temp;
					temp.x = i;
					temp.y = j;
					yellow_points.push_back(temp);
					//for(k=0; k<3; k++) a.at<Vec3b>(i,j)[k] = 0;
				}

				else if(isblue(a,i,j))
				{
					Point2d temp;
					temp.x = i;
					temp.y = j;
					blue_points.push_back(temp);
					//for(k=0; k<3; k++) a.at<Vec3b>(i,j)[k] = 0;
				}
			}

		if(yellow_points.size() == 0) {printf("No yellow points detected\n"); continue;}
		if(blue_points.size() == 0) {printf("No purple points detected\n"); continue;}

		Point2d yellow_centre, blue_centre;
		yellow_centre.x = 0;
		yellow_centre.y = 0;
		blue_centre.x = 0;
		blue_centre.y = 0;

		for(i=0; i<yellow_points.size(); i++)
		{
			yellow_centre.x += yellow_points[i].x;
			yellow_centre.y += yellow_points[i].y;
		}

		yellow_centre.x /= yellow_points.size();
		yellow_centre.y /= yellow_points.size();

		for(i=0; i<blue_points.size(); i++)
		{
			blue_centre.x += blue_points[i].x;
			blue_centre.y += blue_points[i].y;
		}

		blue_centre.x /= blue_points.size();
		blue_centre.y /= blue_points.size();

		Point2d bot_centre;
		bot_centre.x = (yellow_centre.x + blue_centre.x)/2;
		bot_centre.y = (yellow_centre.y + blue_centre.y)/2;

		float bot_angle = atan2(yellow_centre.x - blue_centre.x, yellow_centre.y - blue_centre.y);

		bot_angle *= 180/PI;

		if(dist_two_points(bot_centre,aim) < threshold) aim = graph[++node];

		float line_angle = atan2(aim.x - bot_centre.x, aim.y - bot_centre.y);
		float offset = bot_angle-line_angle;
		if(abs(offset) < 10) {sendCommand('w'); continue;}
		if(offset > 10) {sendCommand('a'); continue;}
		if(offset < -10) sendCommand('d');
	}

}

int main()
{	
	int i,j,k;
	/*VideoCapture cap(1);
	if(!cap.isOpened())
		return -1;
	
	namedWindow("Video",WINDOW_NORMAL);
	Mat frame;
	cap >> frame;
	imshow("Video",frame);
	while(1)
	{
		int flag = waitKey(10);
		if(flag == 27) break;
	}

	Mat src = frame.clone();*/

	Mat a1 = imread("final_uncropped.jpg",1);

	/*namedWindow("Original",WINDOW_NORMAL);
	imshow("Original",src);
	while(1)
	{
		int flag = waitKey(10);
		if(flag == 27) break;
	}*/

	Mat a(ROW2-ROW1,COL2-COL1,CV_8UC3,Scalar(0,0,0));
	for(i=0; i<a.rows; i++)
		for(j=0; j<a.cols; j++)
				for(k=0; k<3; k++)
					a.at<Vec3b>(i,j)[k] = a1.at<Vec3b>(i+ROW1,j+COL1)[k];

	imshow("Cropped",a);
	while(1)
	{
		int flag= waitKey(10);
		if(flag == 27) break;
	}

	Mat original = a.clone();
	//resize(original1, original, Size(), 699/480, 785/460);

	//printf("%d %d",isgreen(a,44,452),isred(a,44,452));

	/*for(i=0; i<a.rows; i++)
		for(j=0; j<a.cols; j++)
		{
			if(isgreen(a,i,j)) printf("green: %d %d\n",i,j);
			if(isred(a,i,j)) printf("red: %d %d\n",i,j);
		}*/	

	Point2d array[2];

	find_src_dest(a,array);

	a = remove_src_dest(a);

	/*namedWindow("Before",WINDOW_NORMAL);
	imshow("Before",a);
	while(1)
	{
		int flag = waitKey(10);
		if(flag == 27) break;
	}*/

	cvtColor(a,a,CV_BGR2GRAY);

	/*namedWindow("After",WINDOW_NORMAL);
	imshow("After",a);
	while(1)
	{
		int flag = waitKey(10);
		if(flag == 27) break;
	}*/

	//a = binary(a);

	/*namedWindow("Before Binary",WINDOW_NORMAL);
	imshow("Before Binary",a);
	while(1)
	{
		int flag = waitKey(10);
		if(flag == 27) break;
	}*/

	Mat b(a.rows,a.cols,CV_8UC1,Scalar(0));

	int low=130,high=20;

	GaussianBlur(a,b,Size(5,5),1.5,1.5);
	Canny(b,b,low,high);

	/*namedWindow("Gauss + Canny",WINDOW_NORMAL);
	imshow("Gauss + Canny",b);
	while(1)
	{
		int flag=waitKey(30);
		if(flag==27) break;
	}*/

	vector<std::vector<cv::Point> > contours;
    findContours(b, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    Mat path = Mat::zeros(b.size(), CV_8UC1);
    drawContours(path, contours, -1, CV_RGB(255,255,255), 1);

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
    Mat kernel = Mat::ones(40,40, CV_8UC1); 
    dilate(path, path, kernel);

    /*imshow("Dilated",path);
    while(1)
    {
        int flag = waitKey(10);
        if(flag == 27) break;
    }*/

    Mat c = path.clone();
	c = binary(c);

	c = isolate_path(c, path);

	/*namedWindow("isolate_path",WINDOW_NORMAL);
	imshow("isolate_path",c);
	while(1)
	{
		int flag = waitKey(10);
		if(flag == 27) break;
	}*/

	//namedWindow("cornerHarris",WINDOW_NORMAL);
	//createTrackbar("Threshold: ", "cornerHarris", &thresh, max_thresh, cornerHarris_demo);

	Mat corners = cornerHarris_demo(c);

	/*imshow("to check",temp_path);
	while(1)
	{
		int flag = waitKey(10);
		if(flag == 27) break;
	}*/

	vector<Point2d> vertices = refine_corners(corners, 20);

	for(i=0; i<vertices.size(); i++)
		printf("(%f,%f)\n",vertices[i].x,vertices[i].y);

	Mat refined(corners.rows, corners.cols, CV_8UC1, Scalar(0));
	for(i=0; i<vertices.size(); i++)
		temp_path.at<uchar>(vertices[i].x,vertices[i].y) = 127;

	/*namedWindow("Refined",WINDOW_NORMAL);
	imshow("Refined",temp_path);
	while(1)
	{
		int flag = waitKey(10);
		if(flag == 27) break;
	}*/

	int **path_matrix = (int **)malloc(vertices.size()*sizeof(int *));
	for(i=0; i<vertices.size(); i++)
		path_matrix[i] = (int *)malloc(vertices.size()*sizeof(int));



	/*for(i=0; i<vertices.size(); i++)
		for(j=i+1; j<vertices.size(); j++)
			if(path_exists(vertices[i],vertices[j],temp_path))
			{
				Point2d start, end;
				start.x = vertices[i].y;
				start.y = vertices[i].x;
				end.x = vertices[j].y;
				end.y = vertices[j].x;
				line(refined,start,end,Scalar(255),3,8,0);
			}

	namedWindow("Final Path", WINDOW_NORMAL);
	imshow("Final Path", refined);
	while(1)
	{
		int flag = waitKey(10);
		if(flag == 27) break;
	}*/

	vector<int> baap_ka_index(vertices.size());

	int start_index, end_index;
	
	int min_dist = INT_MAX;
	for(i=0; i<vertices.size(); i++)
		if(dist_two_points(array[0],vertices[i]) < min_dist) {start_index=i; min_dist = dist_two_points(array[0],vertices[i]);}
	min_dist = INT_MAX;
	for(i=0; i<vertices.size(); i++)
		if(dist_two_points(array[1],vertices[i]) < min_dist) {end_index=i; min_dist = dist_two_points(array[1],vertices[i]);}
	min_dist = INT_MAX;
	int temp;
	for(i=0; i<vertices.size(); i++)
	{
		if(i == end_index) continue;
		if(dist_two_points(vertices[end_index],vertices[i]) < min_dist) {temp=i; min_dist = dist_two_points(vertices[end_index],vertices[i]);}
	}
	end_index = temp;


	//printf("%f %f\n%f %f\n",vertices[start_index].x,vertices[start_index].y,vertices[end_index].x, vertices[end_index].y);

	int *visited = (int *)malloc(vertices.size()*sizeof(int));

	for(i=0; i<vertices.size(); i++)
	{
		visited[i] = 0;
		for(j=0; j<vertices.size(); j++) path_matrix[i][j] = 0;
	}

	path_exists(temp_path,vertices,path_matrix);

	vector<Point2d> graph;
	dijkstra(vertices, start_index, end_index, visited, path_matrix, baap_ka_index);

	draw_final_path(vertices, end_index, start_index, original, baap_ka_index,graph);

	/*namedWindow("Final",WINDOW_NORMAL);
	imshow("Final",original);
	while(1)
	{
		int flag = waitKey(10);
		if(flag == 27) break;
	}*/

	for(i=0; i<graph.size()/2; i++)
	{
		Point2d temp = graph[i];
		graph[i] = graph[graph.size()-1-i];
		graph[graph.size()-1-i] = temp;
	}

	i=-1;
	Mat LEDs = original.clone();
	namedWindow("LEDs",WINDOW_NORMAL);
	while(1)
	{
		if(i==-1) {LEDs = original.clone(); imshow("LEDs",LEDs); waitKey(50); i++;}
		if(i==graph.size()-1) {i = -1; continue;}
		//printf("something %d\n",i);
		Point2d start, end;
		start.x = graph[i].y;
		start.y = graph[i].x;
		end.x = graph[i+1].y;
		end.y = graph[i+1].x;
		line(LEDs,start,end,Scalar(255,255,255),2,8,0);
		imshow("LEDs",LEDs);
		i++;		
		int flag = waitKey(50);
		if(flag==27) break;
	}

	run_bot(graph);

	return 0;
}