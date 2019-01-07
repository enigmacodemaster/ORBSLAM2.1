/*************************************************************************
	> File Name: Encoder.cc
	> Author: 
	> Mail: 
	> Created Time: Sun 23 Sep 2018 03:42:40 PM CST
 ************************************************************************/

#include <iostream>
#include <opencv2/opencv.hpp>

#include "Encoder.h"

using namespace std;

namespace ORB_SLAM2
{
Encoder::Encoder(const std::string& path_to_Enc, const std::string &strSettingPath)
{
	path_to_EncData = path_to_Enc;

	Toc = cv::Mat::eye(4,4,CV_32F);
	Tco = cv::Mat::eye(4,4,CV_32F);
	
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
	cv::FileNode Toc_ = fSettings["Camera.Toc"];

	Roc = (cv::Mat_<float>(3,3) << Toc_[0], Toc_[1], Toc_[2], Toc_[4], Toc_[5], Toc_[6], Toc_[8], Toc_[9], Toc_[10]);
	toc = (cv::Mat_<float>(3,1) << Toc_[3], Toc_[7], Toc_[11]);

	Roc.copyTo(Toc.rowRange(0,3).colRange(0,3));
	toc.copyTo(Toc.rowRange(0,3).col(3));

	Rco = Roc.t();
	tco = -Rco*toc;

	Rco.copyTo(Tco.rowRange(0,3).colRange(0,3));
	tco.copyTo(Tco.rowRange(0,3).col(3));

cout << "loading the para, from encoder to camera, Toc = " << endl << Toc << endl << "Tco =" << endl << Tco << endl ;

}

void Encoder::CacheEncoder()
{
cout << "--- cacheEncoder -----" << endl;
	ifstream filedata;

	filedata.open(path_to_EncData);

	if (!filedata.is_open())
	{
		cerr<< "Cannot open the txt file!" << endl;
	}
	else
	{
		double data;
		int count = 1;
		double timeStamp, v, angle;

		while (!filedata.eof())
		{
    		filedata >> data;
					
			if(count == 1)
			{
				timeStamp = data;

				count++;
			}
			else if(count == 2)
			{
				v = data;
				
				count++;
			}
			else if(count == 3)
			{
				angle = data;

				enc_t.push_back(timeStamp);
				enc_v.push_back(v);
				enc_w.push_back(angle);
				
				count = 1;				
			}
		}
		cout << "Odom read finish! " << endl << "the size of encoder data is: " << enc_t.size() << endl;
	}
}

int Encoder::GetEncoderSize()
{
	if(enc_t.size() == enc_v.size() && enc_v.size() == enc_w.size())
		return enc_t.size();
	else
		return 0;
}

bool Encoder::GetEncoderData(double t1, double t2, std::vector<double>& _enc_t, std::vector<double>& _enc_v, std::vector<double>& _enc_w)
{
	int beginTh = -1, endTh = -1;
	double beginErr = -1, endErr = -1;
//cout << "test2" << endl;
//cout << "t1 = " << t1 << endl;
//cout << "t2 = " << t2 << endl;
	for(std::size_t i=0; i < enc_t.size(); i++)
	{
		if(beginErr < 0 || abs(enc_t[i] - t1) < beginErr)
		{
			beginErr = abs(enc_t[i] - t1);
			beginTh = i;
		}

		if(endErr < 0 || abs(enc_t[i] - t2) < endErr)
		{
			endErr = abs(enc_t[i] - t2);
			endTh = i;
		}
	}
//cout << "test3" << endl;

//cout << "beginTh: " << beginTh << endl << "enc_t[beginTh]:" << enc_t[beginTh] << endl;
//cout << "endTh: " << endTh  << endl << "enc_t[endTh]:" << enc_t[endTh] << endl;

	if(beginTh == -1 || endTh == -1 || endTh - beginTh < 1)
		return false;
	else
	{
		for(int i = beginTh; i <= endTh; i++)
		{
			_enc_t.push_back(enc_t[i]);
			_enc_v.push_back(enc_v[i]);
			_enc_w.push_back(enc_w[i]);
		}

		return true;
	}

}

bool Encoder::UpdataPoseWithEncoder(std::vector<double> _enc_t, std::vector<double> _enc_v, std::vector<double> _enc_w, cv::Mat &Tcw)
{
//cout << "test5" << endl;
//cout << "test55" << endl;

//cout << "Tcw.type()" << Tcw.type() << endl;

	cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
	cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
cv::Mat test = cv::Mat::eye(4,4,CV_32F);
//cout << "test.type()" << test.type() << endl;
//cout << "Rwc.type()" << Rwc.type() << endl;
//cout << "Rco.type()" << Rco.type() << endl;

	cv::Mat Rwo = Rwc*Rco;
//cout << "Rwo =" << endl << Rwo << endl;

	cv::Mat two = Rwc*tco + twc;

//cout << "two =" << endl << two << endl;

//cout << "_enc_t.size = " << endl << _enc_t.size() << endl;
//cout << "_enc_v.size = " << endl << _enc_v.size() << endl;
//cout << "_enc_w.size = " << endl << _enc_w.size() << endl;


	for(std::size_t i=1; i < _enc_t.size(); i++)
	{
		float dt = _enc_t[i] - _enc_t[i-1];
//cout << "dt =" << dt << endl;
		float dw = _enc_w[i-1]*dt;
//cout << "dw =" << dw << endl;
		cv::Mat v = (cv::Mat_<float>(3,1) << _enc_v[i-1], 0, 0);
//cout << "v =" << endl << v << endl;
		cv::Mat dR = (cv::Mat_<float>(3,3) << cos(dw), sin(dw), 0, -sin(dw), cos(dw), 0, 0, 0, 1);
//cout << "dR =" << dR << endl;
		two += Rwo*v*dt;
//cout << "two =" << endl << two << endl;
		Rwo = Rwo*dR; // maybe normal?
//cout << "Rwo =" << endl << Rwo << endl;
	}
//cout << "test6" << endl;

	Rwc = Rwo*Roc;
	twc = Rwo*toc + two;

	cv::Mat Rcw = Rwc.rowRange(0,3).colRange(0,3).t();
	cv::Mat tcw = -Rcw*twc;
	Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
	tcw.copyTo(Tcw.rowRange(0,3).col(3));

	return 1;
}

bool Encoder::UpdataPoseWithEncoder(double t1, double t2, cv::Mat &Tcw)
{
//cout << "test!" << endl;
	std::vector<double> _enc_t, _enc_v, _enc_w;

	bool Enc = GetEncoderData(t1, t2, _enc_t, _enc_v, _enc_w);
//cout << "test4" << endl;

	if(Enc)
	{
		UpdataPoseWithEncoder(_enc_t, _enc_v, _enc_w, Tcw);
		return 1;
	}
	else
	{
//cout << "Enc == 0" << endl;
		return 0;
	}
	

	
}

}