#include "opencv2\aruco.hpp"
#include "opencv2\core.hpp"
#include "opencv2\imgcodecs.hpp"
#include "opencv2\imgproc.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\calib3d.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
#include <windows.h>
#include <cstdio>
#include <algorithm>
#include <time.h>

using namespace std;
using namespace cv;


const float calibrationSquareDimension = 0.0228f; //meters
const float arucoSquareDimension = 0.07965f; //meters
const Size chessboardDimensions = Size( 9, 6);
const int numberOfMarkers = 50;


char firstValue[5] = { 0 };
char secondValue[5] = { 0 };
char lpBuffer[] = "-999 -999";
bool shootingFlag = 0;
int xLength = 15;
const int accuracyRange = 7;



void createArucoMarkers() {
	
	Mat outputMarker;

	Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	for (int i = 0; i < 50; i++) {

		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		ostringstream convert;
		string imageName = "4x4Marker_";
		convert << imageName << i << ".jpg";
		imwrite(convert.str(), outputMarker);
	}
}


void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners) {

	for (int i = 0; i < boardSize.height; i++) {
		for (int j = 0; j < boardSize.width; j++) {
			corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
		}
	}

}


void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults = false) {

	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++) {

		vector<Point2f> pointBuf;
		bool found = findChessboardCorners(*iter, Size(9, 6), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (found) {
			allFoundCorners.push_back(pointBuf);
		}
		
		if (showResults) {
			drawChessboardCorners(*iter, Size(9, 6), pointBuf, found);
			imshow("Looking for corners", *iter);
			waitKey(0);
		}
	}
}


void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients) {

	vector<vector<Point2f>> checkerboardImageSpacePoints;
	getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

	vector<vector<Point3f>> worldSpaceCornerPoints(1);

	createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

	vector<Mat> rVectors, tVectors;
	distanceCoefficients = Mat::zeros(8, 1, CV_64F);

	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);

}


bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients) {

	ofstream outStream(name);
	if (outStream) {

		uint16_t rows = cameraMatrix.rows;
		uint16_t columns = cameraMatrix.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {

				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
				
			}
		}

		rows = distanceCoefficients.rows;
		columns = distanceCoefficients.cols;

		outStream << rows << endl;
		outStream << columns << endl;

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {

				double value = distanceCoefficients.at<double>(r, c);
				outStream << value << endl;

			}
		}

		outStream.close();
		return true;

	}
	return false;
}

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients) {

	ifstream inStream(name);
	if (inStream) {

		uint16_t rows;
		uint16_t columns;

		inStream >> rows;
		inStream >> columns;

		cameraMatrix = Mat(Size(columns, rows), CV_64F);

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {

				double read = 0.0f;
				inStream >> read;
				cameraMatrix.at<double>(r, c) = read;
				//cout << cameraMatrix.at<double>(r, c) << "\n";
			}
		}

		//Distance coefficients

		inStream >> rows;
		inStream >> columns;

		distanceCoefficients = Mat::zeros(rows, columns, CV_64F);

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < columns; c++) {

				double read = 0.0f;
				inStream >> read;
				distanceCoefficients.at<double>(r, c) = read;
				//cout << distanceCoefficients.at<double>(r, c) << "\n";
			}
		}
		
		inStream.close();
		return true;
	}

	return false;
}

void showMarkerStatus(Mat frame, int id, vector<Point2f> markerCenter, double  screenCenterX, double screenCenterY, int trackingMarker) {

	cout << "  " << "";
	cout << id << "\t\t";
	if (markerCenter[id].x != 0) {
		cout << markerCenter[id].x << "\t";
		cout << markerCenter[id].y << "\t\t";
	}
	else {
		cout << "-" << "\t";
		cout << "-" << "\t\t";
	}

	if (markerCenter[id].x != 0) {
		cout << screenCenterX - markerCenter[id].x << "\t";
		cout << screenCenterY - markerCenter[id].y << "\t";
	}
	else {
		cout << "-" << "\t";
		cout << "-" << "\t";
	}

	if (trackingMarker == id && markerCenter[id].x == 0 && shootingFlag == 0) {
		cout << "SEARCHING" << "\t";
		putText(frame, "Searching: marker " + to_string(trackingMarker), Point(10, 50), 1, 3, Scalar(0, 0, 255), 3);
	}
	if (trackingMarker == id && markerCenter[id].x != 0 && shootingFlag == 0) {
		cout << "TRACKING" << "\t";
		putText(frame, "Tracking: marker " + to_string(trackingMarker), Point(10, 50), 1, 3, Scalar(0, 0, 255), 3);
	}
	if (trackingMarker == id && shootingFlag==1) {
		cout << "ELIMINATING" << "\t";
		putText(frame, "Eliminating: marker " + to_string(trackingMarker), Point(10, 50), 1, 3, Scalar(0, 0, 255), 3);
	}
	if (trackingMarker > id)
		cout << "ELIMINATED" << "\t";
	else
		cout << "" << "";
	cout << "" << "\n";

	if(trackingMarker > 3)
		putText(frame, " All targets eliminated", Point(10, 50), 1, 3, Scalar(0, 0, 255), 3);
}

int checkMarkerPosition(int id, vector<Point2f> markerCenter, double screenCenterX, double screenCenterY, int trackingMarker, int *start, int *end, bool *shootingFlag) {


	if ((markerCenter[id].x > (screenCenterX - accuracyRange) && markerCenter[id].x < (screenCenterX + accuracyRange)) &&
		(markerCenter[id].y >(screenCenterY - accuracyRange) && markerCenter[id].y < (screenCenterY + accuracyRange)) &&
		(trackingMarker == id)&&
		(*shootingFlag) == 0) {
		if ((*start == 0)) {
			(*start) = GetTickCount();
			(*end) = GetTickCount();
		}
		else
			(*end) = GetTickCount();

		if ((*end)- (*start) > 1000) {
			(*shootingFlag) = 1;
			(*start) = 0;
			(*end) = 0;
		}
	} else {
		if ((*shootingFlag) == 0) {
			(*start) = 0;
			(*end) = 0;
		}
	}

	if ((*shootingFlag) == 1) {
		if ((*start == 0)) {
			(*start) = GetTickCount();
			(*end) = GetTickCount();
		}
		else
			(*end) = GetTickCount();

		if ((*end) - (*start) >= 500) {
			trackingMarker = trackingMarker + 1;
			(*shootingFlag) = 0;
			(*start) = 0;
			(*end) = 0;
		}
	}

	return trackingMarker;
}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimensions) {

	Mat frame;

	int trackingMarker = 0;
	COORD c;
	c.X = 0;
	c.Y = 0;

	int start = 0;
	int end = 0;


	vector<int> markerIds;
	vector<int> detectedMarkers(numberOfMarkers);
	vector<Point2f> markerCenter(numberOfMarkers);
	vector<vector<Point2f>> markerCorners, rejectedCandidates;

	aruco::DetectorParameters parameters;
	Ptr <aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	VideoCapture vid(0);

	if (!vid.isOpened()) {
		return -1;
	}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	HANDLE hComm;
	TCHAR *pcCommPort = TEXT("COM3");
	DCB dcb;


	hComm = CreateFile(pcCommPort,    //port name
		GENERIC_READ | GENERIC_WRITE, //Read/Write
		0,                            // No Sharing
		NULL,                         // No Security
		OPEN_EXISTING,// Open existing port only
		0,            // Non Overlapped I/O
		NULL);        // Null for Comm Devices

	if (hComm == INVALID_HANDLE_VALUE)
		printf(" Error in opening serial port");
	else
		printf(" Opening serial port successful");

	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), c);
	Sleep(2000);

	
	SecureZeroMemory(&dcb, sizeof(DCB));
	dcb.DCBlength = sizeof(DCB);
	BOOL Status = GetCommState(hComm, &dcb);

	dcb.BaudRate = CBR_9600;  // Setting BaudRate = 9600
	dcb.ByteSize = 8;         // Setting ByteSize = 8
	dcb.StopBits = ONESTOPBIT;// Setting StopBits = 1
	dcb.Parity = NOPARITY;  // Setting Parity = None

	Status = SetCommState(hComm, &dcb);

	Status = GetCommState(hComm, &dcb);

	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50; // in milliseconds
	timeouts.ReadTotalTimeoutConstant = 50; // in milliseconds
	timeouts.ReadTotalTimeoutMultiplier = 10; // in milliseconds
	timeouts.WriteTotalTimeoutConstant = 50; // in milliseconds
	timeouts.WriteTotalTimeoutMultiplier = 10; // in milliseconds

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	namedWindow("Marker tracking", CV_WINDOW_AUTOSIZE);

	int screenWidth = vid.get(CV_CAP_PROP_FRAME_WIDTH);
	int screenHeight = vid.get(CV_CAP_PROP_FRAME_HEIGHT);

	int screenCenterX = screenWidth / 2;
	int screenCenterY = screenHeight / 2;
	Point screenCenter(screenCenterX, screenCenterY);

	

	while (true)
	{
		if (!vid.read(frame))
			break;

		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);

		int i = 0;

		for (int j = 0; j < numberOfMarkers; j++) {
			markerCenter[j].x = 0;
			markerCenter[j].y = 0;
		}

		for (vector<int>::const_iterator k = markerIds.begin(); k != markerIds.end(); ++k) {

			aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

			detectedMarkers[i] = *k;

			markerCenter[*k].x = floor((markerCorners[i][0].x + markerCorners[i][2].x) / 2);
			markerCenter[*k].y = floor((markerCorners[i][0].y + markerCorners[i][2].y) / 2);

			
			if (trackingMarker == *k) {
				if (shootingFlag == 0) {
					circle(frame, markerCenter[*k], 4, Scalar(0, 0, 255), 2);	//red & yellow
					circle(frame, markerCenter[*k], 1, Scalar(0, 255, 255), 2);
				}
				else {

					Point xTopLeft(markerCenter[*k].x - xLength, markerCenter[*k].y - xLength);
					Point xBottomLeft(markerCenter[*k].x - xLength, markerCenter[*k].y + xLength);
					Point xTopRight(markerCenter[*k].x + xLength, markerCenter[*k].y - xLength);
					Point xBottomRight(markerCenter[*k].x + xLength, markerCenter[*k].y + xLength);

					line(frame, xTopLeft, xBottomRight, Scalar(0, 0, 255), 6);
					line(frame, xTopLeft, xBottomRight, Scalar(0, 255, 255), 2);

					line(frame, xBottomLeft, xTopRight, Scalar(0, 0, 255), 6);
					line(frame, xBottomLeft, xTopRight, Scalar(0, 255, 255), 2);

					circle(frame, markerCenter[*k], 4, Scalar(0, 0, 255), 2);	//red & yellow
					circle(frame, markerCenter[*k], 1, Scalar(0, 255, 255), 2);
				}
			}
			else {
				if (trackingMarker < *k) {
					circle(frame, markerCenter[*k], 1, Scalar(0, 0, 255), 2);	//red
					circle(frame, markerCenter[*k], 4, Scalar(0, 0, 255), 2);
				}
				else {
					circle(frame, markerCenter[*k], 1, Scalar(169, 169, 169), 2);	//grey
					circle(frame, markerCenter[*k], 4, Scalar(169, 169, 169), 2);
				}
			}
			i++;
		}

		circle(frame, screenCenter, 20, Scalar(0, 0, 255), 1);
		circle(frame, screenCenter, 10, Scalar(0, 0, 255), 1);

		cout << "Detected markers: " << " \t\t";
		cout << i << "\n";
		cout << "Marker:		Position:		To center:	Status:" << "\n";
		cout << "  ID		X	Y		x	y" << "\n";
		for(int j = 0; j < 4; j++)
			showMarkerStatus(frame, j, markerCenter, screenCenterX, screenCenterY, trackingMarker);

	
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


		memset(lpBuffer, 0, sizeof(lpBuffer));

		if (markerCenter[trackingMarker].x != 0) {
			itoa(screenCenterX - markerCenter[trackingMarker].x, firstValue, 10);
			itoa(screenCenterY - markerCenter[trackingMarker].y, secondValue, 10);
		}
		else {
			itoa(999, firstValue, 10);
			itoa(999, secondValue, 10);
		}
		

		strcat(lpBuffer, firstValue);
		strcat(lpBuffer, " ");
		strcat(lpBuffer, secondValue);
		strcat(lpBuffer, "\0");
		
		
		DWORD dNoOFBytestoWrite;         // No of bytes to write into the port
		DWORD dNoOfBytesWritten = 0;     // No of bytes written to the port
		dNoOFBytestoWrite = sizeof(lpBuffer);

		cout << "\n" << "" ;
		cout << "Output Buffer:" << "\n";
		cout << lpBuffer << "\t\t\t";


		Status = WriteFile(hComm,        // Handle to the Serial port
			lpBuffer,     // Data to be written to the port
			dNoOFBytestoWrite,  //No of bytes to write
			&dNoOfBytesWritten, //Bytes written
			NULL);

		trackingMarker = checkMarkerPosition(trackingMarker, markerCenter, screenCenterX, screenCenterY, trackingMarker, &start, &end, &shootingFlag);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), c);
		imshow("Marker tracking", frame);


		if (waitKey(10) >= 0) {
			memset(lpBuffer, 0, sizeof(lpBuffer));
			strcat(lpBuffer, "998"); 
			strcat(lpBuffer, " ");
			strcat(lpBuffer, "998");
			strcat(lpBuffer, "\0");
			dNoOFBytestoWrite = sizeof(lpBuffer);
				WriteFile(hComm,        // Handle to the Serial port
					lpBuffer,     // Data to be written to the port
					dNoOFBytestoWrite,  //No of bytes to write
					&dNoOfBytesWritten, //Bytes written
					NULL);
			memset(lpBuffer, 0, sizeof(lpBuffer));
			CloseHandle(hComm);//Closing the Serial Port
			break;
		}
	}

	return 1;
}

void cameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients) {

	Mat frame;
	Mat drawToFrame;


	vector<Mat> savedImages;

	vector<vector<Point2f>> markerCorners, rejectedCandidates;

	VideoCapture vid(0);

	if (!vid.isOpened())
		return;

	int framesPerSecond = 20;

	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

	while (true) {

		if (!vid.read(frame))
			break;

		vector<Vec2f> foundPoints;
		bool found = false;

		found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		frame.copyTo(drawToFrame);
		drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);

		if (found)
			imshow("Webcam", drawToFrame);
		else
			imshow("Webcam", frame);
		char character = waitKey(1000 / framesPerSecond);

		switch (character) {
		case ' ':
			if (found) {
				Mat temp;
				frame.copyTo(temp);
				savedImages.push_back(temp);
			}
			break;
		case 13:
			if (savedImages.size() > 20) {
				cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
				saveCameraCalibration("Webcam Calibration", cameraMatrix, distanceCoefficients);
			}
			break;
		case 27:
			return; break;
		}
	}
}


int main(int argv, char** argc) {
	
	
	//createArucoMarkers();

	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distanceCoefficients;

	//cameraCalibrationProcess(cameraMatrix, distanceCoefficients);
	loadCameraCalibration("Webcam Calibration", cameraMatrix, distanceCoefficients);
	startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension);

	return 0;
}