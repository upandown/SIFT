#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_VC1.h"
#include<qfiledialog.h>
#include <string>
#include "highgui/highgui.hpp"  
#include "opencv2/nonfree/nonfree.hpp"  
#include "opencv2/legacy/legacy.hpp" 
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <math.h>
#include <cv.h>
#include <exception>

class VC1 : public QMainWindow
{
	Q_OBJECT

public:
	VC1(QWidget *parent = Q_NULLPTR);
private slots:
	int OnSelctButton1Click(bool check);
	int OnSelctButton2Click(bool check);
	void OnDealButton(bool check);
private:
	Ui::VC1Class ui;
};
