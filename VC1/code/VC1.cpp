#include "VC1.h"
using namespace cv;
using namespace std;
QString filePath1;
QString filePath2;


VC1::VC1(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	connect(ui.selectButton1, SIGNAL(clicked(bool)), this, SLOT(OnSelctButton1Click(bool)));
	connect(ui.selectButton2, SIGNAL(clicked(bool)), this, SLOT(OnSelctButton2Click(bool)));
	connect(ui.dealButton, SIGNAL(clicked(bool)), this, SLOT(OnDealButton(bool)));
}


int VC1::OnSelctButton1Click(bool checked) {
	
	 filePath1 = QFileDialog::getOpenFileName(this);
	//ui.lineEdit->setText(filePath);  //打印文件路径、

	return 0;

}



int VC1::OnSelctButton2Click(bool checked) {

	filePath2 = QFileDialog::getOpenFileName(this);
	//ui.lineEdit->setText(filePath);  //打印文件路径、
	return 0;

}
Point2f getTransformPoint(const Point2f originalPoint, const Mat &transformMaxtri);


//计算原始图像点位在经过矩阵变换后在目标图像上对应位置
Point2f getTransformPoint(const Point2f originalPoint, const Mat &transformMaxtri)
{
	Mat originelP, targetP;
	originelP = (Mat_<double>(3, 1) << originalPoint.x, originalPoint.y, 1.0);
	targetP = transformMaxtri * originelP;
	float x = targetP.at<double>(0, 0) / targetP.at<double>(2, 0);
	float y = targetP.at<double>(1, 0) / targetP.at<double>(2, 0);
	return Point2f(x, y);
}


void VC1::OnDealButton(bool check) {
	try {
		Mat dst1 = imread(filePath1.toStdString().c_str());
		Mat dst2 = imread(filePath2.toStdString().c_str());
		Size size = Size(500, 500);

		Mat image01;
		Mat image02;

		cv::resize(dst1, image01, size);
		cv::resize(dst2, image02, size);
		imshow("拼接图像1", image01);
		imshow("拼接图像2", image02);

		//灰度图转换
		Mat image1, image2;
		/*
		cvtColor(image01, image1, CV_RGB2GRAY);
		cvtColor(image02, image2, CV_RGB2GRAY);*/
		image1 = image01;
		image2 = image02;
		//提取特征点  
		SiftFeatureDetector siftDetector(800);  // 海塞矩阵阈值
		vector<KeyPoint> keyPoint1, keyPoint2;
		siftDetector.detect(image1, keyPoint1);
		siftDetector.detect(image2, keyPoint2);

		//特征点描述，为下边的特征点匹配做准备  
		SiftDescriptorExtractor siftDescriptor;
		Mat imageDesc1, imageDesc2;
		siftDescriptor.compute(image1, keyPoint1, imageDesc1);
		siftDescriptor.compute(image2, keyPoint2, imageDesc2);

		//获得匹配特征点，并提取最优配对  	
		FlannBasedMatcher matcher;
		vector<DMatch> matchePoints;
		matcher.match(imageDesc1, imageDesc2, matchePoints, Mat());


		Mat img_matches;

		drawMatches(image1, keyPoint1, image2, keyPoint2, matchePoints, img_matches);
		//imshow("Match image",img_matches);
		//计算匹配结果中距离最大和距离最小值
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
		//筛选出较好的匹配点
		vector<DMatch> goodMatches;
		for (int m = 0; m < matchePoints.size(); m++)
		{
			if (matchePoints[m].distance < 0.6*max_dist)
			{
				goodMatches.push_back(matchePoints[m]);
			}
		}
		cout << "The number of good matches:" << goodMatches.size() << endl;
		//画出匹配结果
		Mat img_out;
		//红色连接的是匹配的特征点数，绿色连接的是未匹配的特征点数
		//matchColor – Color of matches (lines and connected keypoints). If matchColor==Scalar::all(-1) , the color is generated randomly.
		//singlePointColor – Color of single keypoints(circles), which means that keypoints do not have the matches.If singlePointColor == Scalar::all(-1), the color is generated randomly.
		//CV_RGB(0, 255, 0)存储顺序为R-G-B,表示绿色
		drawMatches(image1, keyPoint1, image2, keyPoint2, goodMatches, img_out, Scalar::all(-1), CV_RGB(0, 0, 255), Mat(), 2);
		imshow("good Matches", img_out);
		//RANSAC匹配过程
		vector<DMatch> m_Matches;
		m_Matches = goodMatches;
		int ptCount = goodMatches.size();
		if (ptCount < 20)
		{
			cout << "Don't find enough match points" << endl;
	
		}

		//坐标转换为float类型
		vector <KeyPoint> RAN_KP1, RAN_KP2;
		//size_t是标准C库中定义的，应为unsigned int，在64位系统中为long unsigned int,在C++中为了适应不同的平台，增加可移植性。
		for (size_t i = 0; i < m_Matches.size(); i++)
		{
			RAN_KP1.push_back(keyPoint1[goodMatches[i].queryIdx]);
			RAN_KP2.push_back(keyPoint2[goodMatches[i].trainIdx]);
			//RAN_KP1是要存储img01中能与img02匹配的点
			//goodMatches存储了这些匹配点对的img01和img02的索引值
		}
		//坐标变换
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
		////求转换矩阵
		//Mat m_homography;
		//vector<uchar> m;
		//m_homography = findHomography(p01, p02, RANSAC);//寻找匹配图像
		//求基础矩阵 Fundamental,3*3的基础矩阵
		vector<uchar> RansacStatus;
		Mat Fundamental = findFundamentalMat(p01, p02, RansacStatus, FM_RANSAC);
		//重新定义关键点RR_KP和RR_matches来存储新的关键点和基础矩阵，通过RansacStatus来删除误匹配点
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
		cout << "RANSAC后匹配点数" << RR_matches.size() << endl;
		Mat img_RR_matches;
		
		drawMatches(image1, RR_KP1, image2, RR_KP2, RR_matches, img_RR_matches);
	
		imshow("After RANSAC", img_RR_matches);

		//sort(RR_matches.begin(), RR_matches.end()); //特征点排序	
		////获取排在前N个的最优匹配特征点
		//vector<Point2f> imagePoints1, imagePoints2;
		//for (int i = 0; i < RR_matches.size()/10; i++)
		//{
		//	
		//	imagePoints1.push_back(RR_KP1[RR_matches[i].queryIdx].pt);
		//	imagePoints2.push_back(RR_KP2[RR_matches[i].trainIdx].pt);
		//}

		//获取排在前N个的最优匹配特征点
		sort(matchePoints.begin(), matchePoints.end()); //特征点排序	
		//获取排在前N个的最优匹配特征点
		vector<Point2f> imagePoints1, imagePoints2;
		for (int i = 0; i < 10; i++)
		{
			imagePoints1.push_back(keyPoint1[matchePoints[i].queryIdx].pt);
			imagePoints2.push_back(keyPoint2[matchePoints[i].trainIdx].pt);
		}

		//获取图像1到图像2的投影映射矩阵，尺寸为3*3
		Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
		Mat adjustMat = (Mat_<double>(3, 3) << 1.0, 0, image01.cols, 0, 1.0, 0, 0, 0, 1.0);
		Mat adjustHomo = adjustMat * homo;

		//获取最强配对点在原始图像和矩阵变换后图像上的对应位置，用于图像拼接点的定位
		Point2f originalLinkPoint, targetLinkPoint, basedImagePoint;
		originalLinkPoint = keyPoint1[matchePoints[0].queryIdx].pt;
		targetLinkPoint = getTransformPoint(originalLinkPoint, adjustHomo);
		basedImagePoint = keyPoint2[matchePoints[0].trainIdx].pt;

		//图像配准
		Mat imageTransform1;
		warpPerspective(image01, imageTransform1, adjustMat*homo, Size(image02.cols + image01.cols + 110, image02.rows));

		//在最强匹配点左侧的重叠区域进行累加，是衔接稳定过渡，消除突变
		Mat image1Overlap, image2Overlap; //图1和图2的重叠部分	
		image1Overlap = imageTransform1(Rect(Point(targetLinkPoint.x - basedImagePoint.x, 0), Point(targetLinkPoint.x, image02.rows)));
		image2Overlap = image02(Rect(0, 0, image1Overlap.cols, image1Overlap.rows));
		Mat image1ROICopy = image1Overlap.clone();  //复制一份图1的重叠部分
		for (int i = 0; i < image1Overlap.rows; i++)
		{
			for (int j = 0; j < image1Overlap.cols; j++)
			{
				double weight;
				weight = (double)j / image1Overlap.cols;  //随距离改变而改变的叠加系数
				image1Overlap.at<Vec3b>(i, j)[0] = (1 - weight)*image1ROICopy.at<Vec3b>(i, j)[0] + weight * image2Overlap.at<Vec3b>(i, j)[0];
				image1Overlap.at<Vec3b>(i, j)[1] = (1 - weight)*image1ROICopy.at<Vec3b>(i, j)[1] + weight * image2Overlap.at<Vec3b>(i, j)[1];
				image1Overlap.at<Vec3b>(i, j)[2] = (1 - weight)*image1ROICopy.at<Vec3b>(i, j)[2] + weight * image2Overlap.at<Vec3b>(i, j)[2];
			}
		}
		Mat ROIMat = image02(Rect(Point(image1Overlap.cols, 0), Point(image02.cols, image02.rows)));	 //图2中不重合的部分
		ROIMat.copyTo(Mat(imageTransform1, Rect(targetLinkPoint.x, 0, ROIMat.cols, image02.rows))); //不重合的部分直接衔接上去
		namedWindow("拼接结果", 0);
		imshow("拼接结果", imageTransform1);
		imwrite("D:\\拼接结果.jpg", imageTransform1);
		waitKey();

	}
	catch (Exception e) {
	
	}
};
