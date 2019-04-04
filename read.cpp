//# include<stdio.h>
//#include <cv.h>
//#include "imgfeatures.h"
//#include <highgui.h>
//#include "sift.h"
//#include "kdtree.h"
//
//
//
//
//#include "utils.h"
//
//#define OPTIONS ":o:m:i:s:c:r:n:b:dxh"
//
//using namespace cv;
///*************************** Function Prototypes *****************************/
//
//static void usage(char*);
//static void arg_parse(int, char**);
//
///******************************** Globals ************************************/
//
//char* pname;
//char* img_file_name;
//char* out_file_name = NULL;
//char* out_img_name = NULL;
//int intvls = SIFT_INTVLS;
//double sigma = SIFT_SIGMA;
//double contr_thr = SIFT_CONTR_THR;
//int curv_thr = SIFT_CURV_THR;
//int img_dbl = SIFT_IMG_DBL;
//int descr_width = SIFT_DESCR_WIDTH;
//int descr_hist_bins = SIFT_DESCR_HIST_BINS;
//int display = 1;
//
//
//#define IMG1_FEAT "features1"
//#define IMG2_FEAT "features2"
//#define IMG_MATCH1 "k-dtree"
//#define  KDTREE_BBF_MAX_NN_CHKS 3
//#define NN_SQ_DIST_RATIO_THR 0.1
//void main() {
//
//	
//
//	IplImage *img1_Feat;
//	IplImage *img2_Feat;
//	IplImage *img1 = cvLoadImage("C:\\Users\\admin\\Pictures\\test1.jpg");
//	IplImage *img2 = cvLoadImage("C:\\Users\\admin\\Pictures\\test2.jpg");
//	struct feature* feat1;
//	struct feature* feat2;
//
//	if (!img1)
//		fprintf(stderr, "error read! \n");
//
//	if (!img2)
//		fprintf(stderr, "error read! \n");
//	
//	img1_Feat = cvCloneImage(img1);//����ͼ1�������������������  
//	img2_Feat = cvCloneImage(img2);//����ͼ2�������������������  
//
//	//Ĭ����ȡ����LOWE��ʽ��SIFT������  
//	//��ȡ����ʾ��1��ͼƬ�ϵ�������  
//	int n1 = 0;
//	int n2 = 0;
//	n1 = sift_features(img1, &feat1);//���ͼ1�е�SIFT������,n1��ͼ1�����������  
//	char filename[] = "feature1.txt";
//	export_features(filename, feat1, n1);//��������������д�뵽�ļ�  
//	draw_features(img1_Feat, feat1, n1);//����������  
//	//cvNamedWindow(IMG1_FEAT);//��������  
//	//cvShowImage(IMG1_FEAT, img1_Feat);//��ʾ  
//
//	//��ȡ����ʾ��2��ͼƬ�ϵ�������  
//	n2 = sift_features(img2, &feat2);//���ͼ2�е�SIFT�����㣬n2��ͼ2�����������  
//	char filename2[] = "feature2.txt";
//	export_features(filename2, feat2, n2);//��������������д�뵽�ļ�  
//	draw_features(img2_Feat, feat2, n2);//����������  
//	//cvNamedWindow(IMG2_FEAT);//��������  
//	//cvShowImage(IMG2_FEAT, img2_Feat);//��ʾ  
//
//	kd_node *kd_root;
//
//	//����ͼ1�������㼯feat1����k-d��������k-d������kd_root  
//	kd_root = kdtree_build(feat1, n1);
//
//    Point pt1, pt2;//���ߵ������˵�  
//	double d0, d1;//feat2��ÿ�������㵽����ںʹν��ڵľ���  
//	int matchNum = 0;//�������ֵ��ɸѡ���ƥ���Եĸ���  
//
//	struct feature* feat;
//	struct feature** nbrs = &feat;
//	CvArr *stacked = img1;
//	//���������㼯feat2�����feat2��ÿ��������feat��ѡȡ���Ͼ����ֵ������ƥ��㣬�ŵ�feat��fwd_match����  
//	for (int i = 0; i < n2; i++)
//	{
//		feat = feat2 + i;//��i���������ָ��  
//		//��kd_root������Ŀ���feat��2������ڵ㣬�����nbrs�У�����ʵ���ҵ��Ľ��ڵ����  
//		int k = kdtree_bbf_knn(kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS);
//		if (k == 2)
//		{
//			d0 = descr_dist_sq(feat, nbrs[0]);//feat������ڵ�ľ����ƽ��  
//			d1 = descr_dist_sq(feat, nbrs[1]);//feat��ν��ڵ�ľ����ƽ��  
//			//��d0��d1�ı�ֵС����ֵNN_SQ_DIST_RATIO_THR������ܴ�ƥ�䣬�����޳�  
//			if (d0 < d1 * NN_SQ_DIST_RATIO_THR)
//			{   //��Ŀ���feat������ڵ���Ϊƥ����  
//				pt2 = Point(cvRound(feat->x), cvRound(feat->y));//ͼ2�е������  
//				pt1 = Point(cvRound(nbrs[0]->x), cvRound(nbrs[0]->y));//ͼ1�е������(feat������ڵ�)  
//				pt2.x += img1->width;//��������ͼ���������еģ�pt2�ĺ��������ͼ1�Ŀ�ȣ���Ϊ���ߵ��յ�  
//				cvLine(stacked, pt1, pt2, CV_RGB(255, 0, 255), 1, 8, 0);//��������  
//				matchNum++;//ͳ��ƥ���Եĸ���  
//				feat2[i].fwd_match = nbrs[0];//ʹ��feat��fwd_match��ָ�����Ӧ��ƥ���  
//			}
//		}
//		free(nbrs);//�ͷŽ�������  
//	}
//	//��ʾ�����澭�����ֵ��ɸѡ���ƥ��ͼ  
//	cvNamedWindow(IMG_MATCH1);//��������  
//	cvShowImage(IMG_MATCH1, stacked);//��ʾ  
//	cvWaitKey(0);
//}


#include "highgui/highgui.hpp"  
#include "opencv2/nonfree/nonfree.hpp"  
#include "opencv2/legacy/legacy.hpp" 
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <math.h>
using namespace cv;
using namespace std;
//����ԭʼͼ���λ�ھ�������任����Ŀ��ͼ���϶�Ӧλ��
Point2f getTransformPoint(const Point2f originalPoint, const Mat &transformMaxtri);

int main( )
{
	Mat dst1 = imread("C:\\Users\\admin\\Pictures\\test3.jpg");
	Mat dst2 = imread("C:\\Users\\admin\\Pictures\\test4.jpg");
	Size size = Size(500,500);

	Mat image01;
	Mat image02;

	resize(dst1, image01, size);
	resize(dst2, image02, size);
	imshow("ƴ��ͼ��1", image01);
	imshow("ƴ��ͼ��2", image02);

	//�Ҷ�ͼת��
	Mat image1, image2;
	cvtColor(image01, image1, CV_RGB2GRAY);
	cvtColor(image02, image2, CV_RGB2GRAY);

	//��ȡ������  
	SiftFeatureDetector siftDetector(800);  // ����������ֵ
	vector<KeyPoint> keyPoint1, keyPoint2;
	siftDetector.detect(image1, keyPoint1);
	siftDetector.detect(image2, keyPoint2);

	//������������Ϊ�±ߵ�������ƥ����׼��  
	SiftDescriptorExtractor siftDescriptor;
	Mat imageDesc1, imageDesc2;
	siftDescriptor.compute(image1, keyPoint1, imageDesc1);
	siftDescriptor.compute(image2, keyPoint2, imageDesc2);

	//���ƥ�������㣬����ȡ�������  	
	FlannBasedMatcher matcher;
	vector<DMatch> matchePoints;
	matcher.match(imageDesc1, imageDesc2, matchePoints, Mat());


	Mat img_matches;
	
	drawMatches(image1, keyPoint1, image2, keyPoint2, matchePoints, img_matches);
	//imshow("Match image",img_matches);
	//����ƥ�����о������;�����Сֵ
	double min_dist = matchePoints[0].distance, max_dist = matchePoints[0].distance;
	for (int m = 0; m < matchePoints.size(); m++)
	{
		if (matchePoints[m].distance < min_dist)
		{
			min_dist = matchePoints[m].distance;
		}
		if (matchePoints[m].distance > max_dist)
		{
			max_dist = matchePoints[m].distance;
		}
	}
	cout << "min dist=" << min_dist << endl;
	cout << "max dist=" << max_dist << endl;
	//ɸѡ���Ϻõ�ƥ���
	vector<DMatch> goodMatches;
	for (int m = 0; m < matchePoints.size(); m++)
	{
		if (matchePoints[m].distance < 0.6*max_dist)
		{
			goodMatches.push_back(matchePoints[m]);
		}
	}
	cout << "The number of good matches:" << goodMatches.size() << endl;
	//����ƥ����
	Mat img_out;
	//��ɫ���ӵ���ƥ���������������ɫ���ӵ���δƥ�����������
	//matchColor �C Color of matches (lines and connected keypoints). If matchColor==Scalar::all(-1) , the color is generated randomly.
	//singlePointColor �C Color of single keypoints(circles), which means that keypoints do not have the matches.If singlePointColor == Scalar::all(-1), the color is generated randomly.
	//CV_RGB(0, 255, 0)�洢˳��ΪR-G-B,��ʾ��ɫ
	drawMatches(image1, keyPoint1, image2, keyPoint2, goodMatches, img_out, Scalar::all(-1), CV_RGB(0, 0, 255), Mat(), 2);
	imshow("good Matches", img_out);
	//RANSACƥ�����
	vector<DMatch> m_Matches;
	m_Matches = goodMatches;
	int ptCount = goodMatches.size();
	if (ptCount < 20)
	{
		cout << "Don't find enough match points" << endl;
		return 0;
	}

	//����ת��Ϊfloat����
	vector <KeyPoint> RAN_KP1, RAN_KP2;
	//size_t�Ǳ�׼C���ж���ģ�ӦΪunsigned int����64λϵͳ��Ϊlong unsigned int,��C++��Ϊ����Ӧ��ͬ��ƽ̨�����ӿ���ֲ�ԡ�
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		RAN_KP1.push_back(keyPoint1[goodMatches[i].queryIdx]);
		RAN_KP2.push_back(keyPoint2[goodMatches[i].trainIdx]);
		//RAN_KP1��Ҫ�洢img01������img02ƥ��ĵ�
		//goodMatches�洢����Щƥ���Ե�img01��img02������ֵ
	}
	//����任
	vector <Point2f> p01, p02;
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		p01.push_back(RAN_KP1[i].pt);
		p02.push_back(RAN_KP2[i].pt);
	}
	/*vector <Point2f> img1_corners(4);
	img1_corners[0] = Point(0,0);
	img1_corners[1] = Point(img_1.cols,0);
	img1_corners[2] = Point(img_1.cols, img_1.rows);
	img1_corners[3] = Point(0, img_1.rows);
	vector <Point2f> img2_corners(4);*/
	////��ת������
	//Mat m_homography;
	//vector<uchar> m;
	//m_homography = findHomography(p01, p02, RANSAC);//Ѱ��ƥ��ͼ��
	//��������� Fundamental,3*3�Ļ�������
	vector<uchar> RansacStatus;
	Mat Fundamental = findFundamentalMat(p01, p02, RansacStatus, FM_RANSAC);
	//���¶���ؼ���RR_KP��RR_matches���洢�µĹؼ���ͻ�������ͨ��RansacStatus��ɾ����ƥ���
	vector <KeyPoint> RR_KP1, RR_KP2;
	vector <DMatch> RR_matches;
	int index = 0;
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		if (RansacStatus[i] != 0)
		{
			RR_KP1.push_back(RAN_KP1[i]);
			RR_KP2.push_back(RAN_KP2[i]);
			m_Matches[i].queryIdx = index;
			m_Matches[i].trainIdx = index;
			RR_matches.push_back(m_Matches[i]);
			index++;
		}
	}
	cout << "RANSAC��ƥ�����" << RR_matches.size() << endl;
	Mat img_RR_matches;
	drawMatches(image1, RR_KP1, image2, RR_KP2, RR_matches, img_RR_matches);
	imshow("After RANSAC", img_RR_matches);

	//sort(RR_matches.begin(), RR_matches.end()); //����������	
	////��ȡ����ǰN��������ƥ��������
	//vector<Point2f> imagePoints1, imagePoints2;
	//for (int i = 0; i < RR_matches.size()/10; i++)
	//{
	//	
	//	imagePoints1.push_back(RR_KP1[RR_matches[i].queryIdx].pt);
	//	imagePoints2.push_back(RR_KP2[RR_matches[i].trainIdx].pt);
	//}
	sort(matchePoints.begin(), matchePoints.end()); //����������	
	//��ȡ����ǰN��������ƥ��������
	vector<Point2f> imagePoints1, imagePoints2;
	for (int i = 0; i < 10; i++)
	{
		imagePoints1.push_back(keyPoint1[matchePoints[i].queryIdx].pt);
		imagePoints2.push_back(keyPoint2[matchePoints[i].trainIdx].pt);
	}

	//��ȡͼ��1��ͼ��2��ͶӰӳ����󣬳ߴ�Ϊ3*3
	Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
	Mat adjustMat = (Mat_<double>(3, 3) << 1.0, 0, image01.cols, 0, 1.0, 0, 0, 0, 1.0);
	Mat adjustHomo = adjustMat * homo;
	// 
	//Point2f originalLinkPoint, targetLinkPoint, basedImagePoint;
	//originalLinkPoint = keyPoint1[matchePoints[0].queryIdx].pt;
	//targetLinkPoint = getTransformPoint(originalLinkPoint, adjustHomo);
	//basedImagePoint = keyPoint2[matchePoints[0].trainIdx].pt;

	////ͼ����׼
	//Mat imageTransform1;
	//warpPerspective(image01, imageTransform1, adjustMat*homo, Size(image02.cols + image01.cols + 10, image02.rows));

	////����ǿƥ����λ�ô��νӣ���ǿƥ��������ͼ1���Ҳ���ͼ2������ֱ���滻ͼ���νӲ��ã�������ͻ��
	//Mat ROIMat = image02(Rect(Point(basedImagePoint.x, 0), Point(image02.cols, image02.rows))); 
	//ROIMat.copyTo(Mat(imageTransform1, Rect(targetLinkPoint.x, 0, image02.cols - basedImagePoint.x, image02.rows)));

	//namedWindow("ƴ�ӽ��", 0);
	//imshow("ƴ�ӽ��", imageTransform1);



	//��ȡ��ǿ��Ե���ԭʼͼ��;���任��ͼ���ϵĶ�Ӧλ�ã�����ͼ��ƴ�ӵ�Ķ�λ
	Point2f originalLinkPoint, targetLinkPoint, basedImagePoint;
	originalLinkPoint = keyPoint1[matchePoints[0].queryIdx].pt;
	targetLinkPoint = getTransformPoint(originalLinkPoint, adjustHomo);
	basedImagePoint = keyPoint2[matchePoints[0].trainIdx].pt;

	//ͼ����׼
	Mat imageTransform1;
	warpPerspective(image01, imageTransform1, adjustMat*homo, Size(image02.cols + image01.cols + 110, image02.rows));

	//����ǿƥ��������ص���������ۼӣ����ν��ȶ����ɣ�����ͻ��
	Mat image1Overlap, image2Overlap; //ͼ1��ͼ2���ص�����	
	image1Overlap = imageTransform1(Rect(Point(targetLinkPoint.x - basedImagePoint.x, 0), Point(targetLinkPoint.x, image02.rows)));
	image2Overlap = image02(Rect(0, 0, image1Overlap.cols, image1Overlap.rows));
	Mat image1ROICopy = image1Overlap.clone();  //����һ��ͼ1���ص�����
	for (int i = 0; i < image1Overlap.rows; i++)
	{
		for (int j = 0; j < image1Overlap.cols; j++)
		{
			double weight;
			weight = (double)j / image1Overlap.cols;  //�����ı���ı�ĵ���ϵ��
			image1Overlap.at<Vec3b>(i, j)[0] = (1 - weight)*image1ROICopy.at<Vec3b>(i, j)[0] + weight * image2Overlap.at<Vec3b>(i, j)[0];
			image1Overlap.at<Vec3b>(i, j)[1] = (1 - weight)*image1ROICopy.at<Vec3b>(i, j)[1] + weight * image2Overlap.at<Vec3b>(i, j)[1];
			image1Overlap.at<Vec3b>(i, j)[2] = (1 - weight)*image1ROICopy.at<Vec3b>(i, j)[2] + weight * image2Overlap.at<Vec3b>(i, j)[2];
		}
	}
	Mat ROIMat = image02(Rect(Point(image1Overlap.cols, 0), Point(image02.cols, image02.rows)));	 //ͼ2�в��غϵĲ���
	ROIMat.copyTo(Mat(imageTransform1, Rect(targetLinkPoint.x, 0, ROIMat.cols, image02.rows))); //���غϵĲ���ֱ���ν���ȥ
	namedWindow("ƴ�ӽ��", 0);
	imshow("ƴ�ӽ��", imageTransform1);
	imwrite("D:\\ƴ�ӽ��.jpg", imageTransform1);
	waitKey();
	return 0;
}

//����ԭʼͼ���λ�ھ�������任����Ŀ��ͼ���϶�Ӧλ��
Point2f getTransformPoint(const Point2f originalPoint, const Mat &transformMaxtri)
{
	Mat originelP, targetP;
	originelP = (Mat_<double>(3, 1) << originalPoint.x, originalPoint.y, 1.0);
	targetP = transformMaxtri * originelP;
	float x = targetP.at<double>(0, 0) / targetP.at<double>(2, 0);
	float y = targetP.at<double>(1, 0) / targetP.at<double>(2, 0);
	return Point2f(x, y);
}
