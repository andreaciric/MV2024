#include "stdafx.h"

#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <iomanip>
#include "Serial.h"
#include "MV_2022_SortingLine_Template.h"

#include <queue>

// Setting for using Basler GigE cameras.
#include <pylon/gige/BaslerGigEInstantCamera.h>
//typedef Pylon::CBaslerGigEInstantCamera Camera_t;
typedef Pylon::CInstantCamera Camera_t;

//using namespace Basler_GigECameraParams;
// Namespace for using pylon objects.
using namespace Pylon;
using namespace cv;
using namespace std;

// Namespace for using GenApi objects
using namespace GenApi;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 100;

struct SortingLineObject{
	int beginning_position;
	int end_position;
};

std::queue<SortingLineObject> objects_queue;

#define IN_OBJECT 1
#define OUT_OF_OBJECT 0
int scanning_status = OUT_OF_OBJECT;

#define WAITING_ON_OBJECT 1
#define IDLE 0
int acquisition_status = IDLE;

Mat image;
CPylonImage image2;
Mat cv_img, gray;
tstring com = _T("\\\\.\\COM20");
char stop[] = "q";
tstring commPortName(com);
Serial serial(commPortName, 57600);
//unsigned char comp[832*832];
unsigned char *iluminationCompR, *iluminationCompG, *iluminationCompB;
Mat grayBackgroundImage;
unsigned long mavis_position = 1000000;
unsigned long mavis_position_enc = 1500;
unsigned long mavis_obj_beginning_position = 0;
unsigned long mavis_obj_beginning_position_past = 1;
unsigned long mavis_obj_end_position = 0;
unsigned long mavis_obj_end_position_past = 1;
unsigned long mavis_obj_position_push = 0;
int mavis_inp = 0;
int in_processing = 0;
int pusher_selection = 0;

int mavis_position_enc_high = 0;
int mavis_position_enc_low = 0;
std::mutex com_lock_dummy;//koristan ako vise threadova treba da komunicira sa hardverom konkurentno

unsigned long foto_camera_distance = 3000;//udaljenost od fotocelije do kamere

int main(int argc, char** argv)
{
	if (argc != 2)
	{
		cout << " Usage: display_image ImageToLoadAndDisplay" << endl;
		return -1;
	}

	image = imread(argv[1], IMREAD_COLOR); // Read the file

	if (!image.data) // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}
	namedWindow("Display window NASA", WINDOW_AUTOSIZE); // Create a window for display.
	imshow("Display window NASA", image); // Show our image inside it.


	namedWindow("Display window CURRENT", WINDOW_NORMAL); // Create a window for display.
	namedWindow("Display window FINAL", WINDOW_NORMAL); // Create a window for display.

	int exitCode = 0;

	// Before using any pylon methods, the pylon runtime must be initialized. 
	PylonInitialize();


	try
	{

		cout << "Creating Camera..." << endl;
		CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());
		cout << "Camera Created." << endl;
		// Print the model name of the camera.
		cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;

		INodeMap& nodemap = camera.GetNodeMap();
		camera.Open();

		// Get camera device information.
		cout << "Camera Device Information" << endl
			<< "=========================" << endl;
		cout << "Vendor           : "
			<< CStringPtr(nodemap.GetNode("DeviceVendorName"))->GetValue() << endl;
		cout << "Model            : "
			<< CStringPtr(nodemap.GetNode("DeviceModelName"))->GetValue() << endl;
		cout << "Firmware version : "
			<< CStringPtr(nodemap.GetNode("DeviceFirmwareVersion"))->GetValue() << endl << endl;
		// Camera settings.
		cout << "Camera Device Settings" << endl
			<< "======================" << endl;


		// GENICAM standard - definisanje parametara kamere
		// Get the integer nodes describing the AOI.
		CIntegerPtr width = nodemap.GetNode("Width");
		CIntegerPtr height = nodemap.GetNode("Height");
		CIntegerPtr offsetX(nodemap.GetNode("OffsetX"));
		CIntegerPtr offsetY(nodemap.GetNode("OffsetY"));
		CEnumerationPtr TriggerMode(nodemap.GetNode("TriggerMode"));
		CEnumerationPtr ExposureMode(nodemap.GetNode("ExposureMode"));
		CFloatPtr ExposureTimeAbs(nodemap.GetNode("ExposureTimeAbs"));

		//podesavanje ROI-ja
		int64_t newWidth = 1600;
		int64_t newHeight = 1200;
		offsetX->SetValue(0);
		offsetY->SetValue(0);
		width->SetValue(newWidth);
		height->SetValue(newHeight);

		TriggerMode->FromString("Off");//trigger je interni
		ExposureMode->FromString("Timed");//triger je odredjen internom vremenskom bazom
		ExposureTimeAbs->SetValue(300);//vreme ekspozicije

		//parametri koji odredjuju nacin akvizicije u kameri
		camera.MaxNumBuffer = 1;
		camera.OutputQueueSize = 1;		
		camera.StartGrabbing(GrabStrategy_UpcomingImage);

		//inicijalizacija pocetka i kraja objekta 
		mavis_obj_beginning_position = SortingLineGetObjectBeginningPosition(&com_lock_dummy);
		mavis_obj_beginning_position_past = mavis_obj_beginning_position;
		mavis_obj_end_position = SortingLineGetObjectEndPosition(&com_lock_dummy);
		mavis_obj_end_position_past = mavis_obj_end_position;
		serial.MavisSendComData(&com_lock_dummy, 23, 1);//INIT- ne dirati
		serial.MavisSendComData(&com_lock_dummy, 7, 40);//neko podesavanje sortirne linije - ne dirati
		serial.MavisSendComData(&com_lock_dummy, 5, 0);//neko podesavanje sortirne linije - ne dirati
		serial.MavisSendComData(&com_lock_dummy, 26, 1);//#0-encoder feedback, 1- servo_feedback- ne dirati
		serial.MavisSendComData(&com_lock_dummy, 25, 0);//tolerance - ne dirati
		serial.MavisSendComData(&com_lock_dummy, 46, 1);//light on- ne dirati
		serial.MavisSendComData(&com_lock_dummy, 13, 600);//pusher time- ne dirati
		serial.MavisSendComData(&com_lock_dummy, 23, 10);//GO - ne dirati 

		while (camera.IsGrabbing())
		{
			mavis_position_enc = SortingLineGetCurrentPosition(&com_lock_dummy);//ocitavanje trenutne pozicije trake

			//procesiranje signala fotocelije koji ukazuju na postojanje objekta
			if (scanning_status == OUT_OF_OBJECT) {
				mavis_obj_beginning_position = SortingLineGetObjectBeginningPosition(&com_lock_dummy);
				if (mavis_obj_beginning_position != mavis_obj_beginning_position_past) {
					//pocetak novog objekta
					scanning_status = IN_OBJECT;
					mavis_obj_beginning_position_past = mavis_obj_beginning_position;
					cout << endl << "OBJECT BEGINNING AT " << mavis_obj_beginning_position << endl;
				}
			}
			else {//IN_OBJECT
				mavis_obj_end_position = SortingLineGetObjectEndPosition(&com_lock_dummy);
				if (mavis_obj_end_position != mavis_obj_end_position_past) {
					//kraj novog objekta
					scanning_status = OUT_OF_OBJECT;
					mavis_obj_end_position_past = mavis_obj_end_position;
					SortingLineObject new_object;
					new_object.beginning_position = mavis_obj_beginning_position;
					new_object.end_position = mavis_obj_end_position;
					objects_queue.push(new_object);
					cout << endl << "OBJECT END AT " << mavis_obj_end_position << endl;
					//cout << "queue size: " << objects_queue.size() << endl;
				}
			}

			//ispitivanje da li postoji novi objekat na traci sa snimljenom i pocetnom i krajnjom pozicijom
			//ako postoji zadaje se pozicioniranje na sredinu snimljenog objekta
			//ako ne postoji azurira se zadata pozicija na 500 impulsa veca od trenutne
			if (objects_queue.size() > 0) {
				SortingLineObject new_object;
				new_object = objects_queue.front();
				//cout << "queue size: " << objects_queue.size() << endl;				
				mavis_position = ((new_object.beginning_position + new_object.end_position) / 2) + foto_camera_distance;
				acquisition_status = WAITING_ON_OBJECT;	
				cout << "W" << mavis_position << endl;
			}
			else {
				mavis_position = mavis_position_enc + 500;//zadata pozicija je uvek za 500 ispred trenutne - sargarepa i konj 
				cout << ".";
			}

			SortingLineSetPosition(&com_lock_dummy, mavis_position);//postavlja se pozicija zaustavljanja izracunata u prethodnom koraku

			//ukoliko se ceka na objekat koji je detektovan na traci, ceka se na zavrseno pozicioniranje
			if (acquisition_status == WAITING_ON_OBJECT) {
				if (SortingLineGetInPositionStatus(&com_lock_dummy) == 1) {//ako je zadata pozicija dostignuta, ide se na akviziciju
					cout << endl  << "IMAGE ACQUISITION AT " << endl;
					AcquireImage(&camera);
					SortingLineObject new_object;
					new_object = objects_queue.front();					
					mavis_obj_position_push = (new_object.beginning_position+ new_object.end_position)/2;//odredjuje se sredina objekta
					pusher_selection = 2;
					switch (pusher_selection) {//rotira u gkur redni broj izbacivaca
					case 0:
						SortingLineSetPositionPusher1(mavis_obj_position_push);
						break;
					case 1:
						SortingLineSetPositionPusher2(mavis_obj_position_push);
						break;
					case 2:
						SortingLineSetPositionPusher3(mavis_obj_position_push);
						break;
					}
					objects_queue.pop();//brise se objekat iz Queue-a
					acquisition_status = IDLE;
				}
			}
			//dodatni ispis
			//cout << "current, start, end:" << mavis_position_enc << std::setw(10) << mavis_obj_beginning_position << std::setw(10) << mavis_obj_end_position << std::setw(10) << mavis_inp << std::endl;
			if (waitKey(30) >= 0) break;
		}
		camera.StopGrabbing();
	}
	catch (const GenericException &e)
	{
		// Error handling.
		cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
		exitCode = 1;
	}
	// Releases all pylon resources. 
	PylonTerminate();
	return exitCode;

}




int AcquireImage(Camera_t *camera) {
	// This smart pointer will receive the grab result data.
	CGrabResultPtr ptrGrabResult;
	CPylonImage imagePylonTemp;
	static Mat rgbImage, grayImage;
	Mat rgbImageWithoutBackground;
	static Mat rgbFinalImage, grayFinalImage, absdifference, difference, rgbImageWithBackground, grayImageWithBackground;
	CImageFormatConverter fc;
	fc.OutputPixelFormat = PixelType_BGR8packed;
	double t = (double)getTickCount();
	camera->RetrieveResult(3000, ptrGrabResult, TimeoutHandling_ThrowException);//INFINITE			
																				// Image grabbed successfully?
	if (ptrGrabResult->GrabSucceeded())
	{
		//converto from Pylon to OpenCV image format		
		fc.Convert(imagePylonTemp, ptrGrabResult);
		rgbImageWithBackground = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)imagePylonTemp.GetBuffer());
		imwrite("../images/rgbImageWithBackground.bmp", rgbImageWithBackground);
		imshow("Display window FINAL", rgbImageWithBackground); // Show our image inside it.
	}
	else
	{
		cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
	}
	return 0;
}


int SortingLineSetPosition(std::mutex* comm_lock, unsigned long position) {
	serial.MavisSendComData(comm_lock, 30, (mavis_position >> 16) & 0xFFFF);//komanda koja postavlja visih 16 bita 32-bitne zadate pozicije
	serial.MavisSendComData(comm_lock, 31, mavis_position & 0xFFFF);//komanda koja postavlja nizih 16 bita 32-bitne zadate pozicije
	return 0;
}

unsigned long SortingLineGetCurrentPosition(std::mutex* comm_lock) {
	unsigned long mavis_position_enc;
	int mavis_position_enc_high, mavis_position_enc_low;
	int retval1, retval2;
	retval1=serial.MavisGetComData(comm_lock, 30, &mavis_position_enc_high);//komanda koja ocitava visih 16 bita 32-bitne trenutne pozicije
	retval2=serial.MavisGetComData(comm_lock, 31, &mavis_position_enc_low);//komanda koja ocitava nizih 16 bita 32-bitne trenutne pozicije
	mavis_position_enc = (mavis_position_enc_high << 16) | mavis_position_enc_low;//trenutna 32-bitna pozicija trake
	return mavis_position_enc;
}

unsigned long SortingLineGetObjectBeginningPosition(std::mutex* comm_lock) {
	unsigned long mavis_position_obj_start;
	int mavis_position_enc_high, mavis_position_enc_low;
	serial.MavisGetComData(comm_lock, 34, &mavis_position_enc_high);//komanda koja ocitava visih 16 bita 32-bitne pozicije pocetka objekta koji je detektovala fotocelija
	serial.MavisGetComData(comm_lock, 35, &mavis_position_enc_low);//komanda koja ocitava nizih 16 bita 32-bitne pozicije pocetka objekta koji je detektovala fotocelija
	mavis_position_obj_start = (mavis_position_enc_high << 16) | mavis_position_enc_low;//trenutna 32-bitna pozicija pocetka objekta
	return mavis_position_obj_start;
}

unsigned long SortingLineGetObjectEndPosition(std::mutex* comm_lock) {
	unsigned long mavis_position_obj_stop;
	int mavis_position_enc_high, mavis_position_enc_low;
	serial.MavisGetComData(comm_lock, 36, &mavis_position_enc_high);//komanda koja ocitava visih 16 bita 32-bitne pozicije pocetka objekta koji je detektovala fotocelija
	serial.MavisGetComData(comm_lock, 37, &mavis_position_enc_low);//komanda koja ocitava nizih 16 bita 32-bitne pozicije pocetka objekta koji je detektovala fotocelija
	mavis_position_obj_stop = (mavis_position_enc_high << 16) | mavis_position_enc_low;//trenutna 32-bitna pozicija pocetka objekta
	return mavis_position_obj_stop;
}

int SortingLineGetInPositionStatus(std::mutex* comm_lock) {
	int mavis_inp;
	serial.MavisGetComData(comm_lock, 32, &mavis_inp);
	return mavis_inp;
}

int SortingLineSetPositionPusher1(unsigned long mavis_position_obj_push) {
	mavis_position_obj_push = mavis_position_obj_push + 7900;
	serial.MavisSendComData(&com_lock_dummy, 40, (mavis_position_obj_push >> 16) & 0xFFFF);//
	serial.MavisSendComData(&com_lock_dummy, 41, mavis_position_obj_push & 0xFFFF);//
	return 0;
}

int SortingLineSetPositionPusher2(unsigned long mavis_position_obj_push) {
	mavis_position_obj_push = mavis_position_obj_push + 12000;
	serial.MavisSendComData(&com_lock_dummy, 42, (mavis_position_obj_push >> 16) & 0xFFFF);//
	serial.MavisSendComData(&com_lock_dummy, 43, mavis_position_obj_push & 0xFFFF);//
	return 0;
}

int SortingLineSetPositionPusher3(unsigned long mavis_position_obj_push) {
	mavis_position_obj_push = mavis_position_obj_push + 14500;
	serial.MavisSendComData(&com_lock_dummy, 44, (mavis_position_obj_push >> 16) & 0xFFFF);//
	serial.MavisSendComData(&com_lock_dummy, 45, mavis_position_obj_push & 0xFFFF);//
	return 0;
}



