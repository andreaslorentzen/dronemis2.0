#include "QR.h"

using namespace cv;
using namespace std;
using namespace zbar;

QR::QR(CV_Handler *cv) {
    initializeQR();
    cvHandler = cv;
}

//#define DEBUG 1
#define AVERAGE_COUNT 6

DronePos QR::checkQR(void) {

    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    namedWindow("MyVideo", CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

    cvHandler->cascadeMutex.lock();

    cv::Mat img(cvHandler->storedImage.size().y,
                cvHandler->storedImage.size().x,
                CV_8UC3,
                cvHandler->storedImage.data());

    cvHandler->cascadeMutex.unlock();

    Mat grey;
    cvtColor(img, grey, CV_BGR2GRAY);
    int width = img.cols;
    int height = img.rows;
    uchar *raw = (uchar *) grey.data;

    // wrap image data
    Image image(width, height, "Y800", raw, width * height);


    // scan the image for barcodes
    int numberQR = scanner.scan(image);                    // Returns number of codes in the image.
    if (numberQR == 1) {
        if (averageCount == AVERAGE_COUNT) averageCount = 1;
        else averageCount++;
    }
    // cout << "Number of QR codes in the image is " << n << endl;
    int x0, x1, x2, x3, y0, y1, y2, y3, xsize, ysize, xmidt, QRsize;

    // extract results
    for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
        vector<Point> vp;

        // do something useful with results
        //cout << "decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' << " " << endl;
        int n = symbol->get_location_size();        // Returns 4 if QR code is scanned
        for (int i = 0; i < n; i++) {
            vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));      //Update VP
            //cout << "symbol->get_location_x = " << symbol->get_location_x(i) << endl;
            //cout << "symbol->get_location_y = " << symbol->get_location_y(i) << endl;
        }

        x0 = symbol->get_location_x(0);    //
        x1 = symbol->get_location_x(1);    //
        x2 = symbol->get_location_x(2);    //
        x3 = symbol->get_location_x(3);    //  Saves the size of the QR code being detected
        y0 = symbol->get_location_y(0);    //
        y1 = symbol->get_location_y(1);    //
        y2 = symbol->get_location_y(2);    //
        y3 = symbol->get_location_y(3);    //
        xsize = (abs((x0 - x2)) + abs((x1 - x3))) / 2;      // Vertical size of the QR
        ysize = (abs((y0 - y2)) + abs((y3 - y1))) / 2;      // Horizontal size of the QR
        xmidt = (x0 + x1 + x2 + x3) / 4;           // The horizontal middle of the QR

        QRsize = (xsize + ysize) / 2;            // The size of the QR
        int distancetoQR = calculateDistanceToQR(QRsize);

        double yleft, yright, ytemp;
        double yratio;

        yleft = y1 - y0;
        yright = y2 - y3;

        if (yleft > yright) {
            direction = 1;                                                      // If direction is 1 - Drone is left of the QR code
        }
        else if (yleft == yright) {
            direction = 0;                                                      // If direction is 0 - Drone is in front of QR code
        }
        else {
            direction = -1;                                                     // If direction is -1, Drone is right of the QR code.
            ytemp = yleft;
            yleft = yright; // Swap
            yright = ytemp;
        }

        yratio = yleft / yright;
        yRatioTemp = yRatioTemp + yratio;
        //cout << "averageCount = " << averageCount << " and yratio = " << yratio << endl;

        if (averageCount == AVERAGE_COUNT) {
            yRatioAverage = yRatioTemp / AVERAGE_COUNT;
            //cout << "yRatioTemp = " << yRatioTemp << endl;
            yRatioTemp = 0;
            y1Diversion = (yRatioAverage * 360.0395) - 359.2821;
            y2Diversion = (yRatioAverage * 637.3656) - 642.2072;
            cout << "Distance = " << calculateDistanceToQR(QRsize) << "cm, with the text: " << symbol->get_data() <<
            endl << endl;
            double xDistanceStatic = 4.208955224; //
            int xDistance = ((xmidt - 320) * 0.7 / xDistanceStatic * distancetoQR / 100);   //
            cout << "Kamera center er: " << xDistance << "cm til venstre for QR-koden" << endl;
            //cout << "y1Diversion (Parallel) = " << y1Diversion << endl;
            //cout << "y2Diversion (Kig på QR)= " << y2Diversion << endl;
            cout << "yRatioAverage = " << yRatioAverage << endl;

            if (yRatioAverage < 1.06) {
                yDiversionAngle = (y1Diversion + y2Diversion) / 2 * direction;
            }
            else {
                yDiversionAngle = y2Diversion * direction;
            }

            DronePosition.x = ((distancetoQR * 0.8 * std::sin(yDiversionAngle * (M_PI / 180))) +
                               xDistance); // xDistance (Forskydning)
            DronePosition.y = (distancetoQR * std::cos(yDiversionAngle * (M_PI / 180)));

            cout << "Droneposition(x,y) = " << DronePosition.x << "," << DronePosition.y << endl;

            //**************************************
            //**** Match up with QR coordinates ****
            //**************************************

            calculateFinalDronePostition(symbol->get_data());
            //cout << symbol->get_data() << endl;

            cout << "FinalDroneposition(x,y) = " << FinalDronePosition.x << "," << FinalDronePosition.y << endl;
            cout << "DroneHeading = " << FinalDronePosition.heading << endl;
            break;
        }
#ifdef DEBUG
        RotatedRect r = minAreaRect(vp);
        Point2f pts[4];
        r.points(pts);
        for(int i=0;i<4;i++){
            line(img,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);
        }
        //cout<<"Angle: "<<r.angle<<endl;
#endif
    }
#ifdef DEBUG
    imshow("MyVideo", img); //show the frame in "MyVideo" window
    if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
        cout << "esc key is pressed by user" << endl;
        break;
    }
#endif
    return FinalDronePosition;
}


void QR::calculateFinalDronePostition(std::string QRname) {
    if (QRname.find("W00") == 0) {       // Wall 0 has been found
        if (QRname.find("W00.00") == 0) {     // Code W00.00
            FinalDronePosition.x = (QRWallCode[0].x - DronePosition.x);
            FinalDronePosition.y = (QRWallCode[0].y - DronePosition.y);
            FinalDronePosition.heading = yDiversionAngle;

        }
        else if (QRname.find("W00.01") == 0) {     // Code W00.01
            FinalDronePosition.x = (QRWallCode[1].x - DronePosition.x);
            FinalDronePosition.y = (QRWallCode[1].y - DronePosition.y);
            FinalDronePosition.heading = yDiversionAngle;
        }
        else if (QRname.find("W00.02") == 0) {     // Code W00.02
            FinalDronePosition.x = (QRWallCode[2].x - DronePosition.x);
            FinalDronePosition.y = (QRWallCode[2].y - DronePosition.y);
            FinalDronePosition.heading = yDiversionAngle;
        }
        else if (QRname.find("W00.03") == 0) {     // Code W00.03
            FinalDronePosition.x = (QRWallCode[3].x - DronePosition.x);
            FinalDronePosition.y = (QRWallCode[3].y - DronePosition.y);
            FinalDronePosition.heading = yDiversionAngle;
        }
        else if (QRname.find("W00.04") == 0) {     // Code W00.04
            FinalDronePosition.x = (QRWallCode[4].x - DronePosition.x);
            FinalDronePosition.y = (QRWallCode[4].y - DronePosition.y);
            FinalDronePosition.heading = yDiversionAngle;
        }
    }

    else if (QRname.find("W01.") == 0) {       // Wall 1 has been found
        if (QRname.find("W01.00") == 0) {     // Code W01.00
            FinalDronePosition.x = (QRWallCode[5].x - DronePosition.y);
            FinalDronePosition.y = (QRWallCode[5].y + DronePosition.x);
            FinalDronePosition.heading = 90 + yDiversionAngle;
        }
        else if (QRname.find("W01.01") == 0) {     // Code W01.01
            FinalDronePosition.x = (QRWallCode[6].x - DronePosition.y);
            FinalDronePosition.y = (QRWallCode[6].y + DronePosition.x);
            FinalDronePosition.heading = 90 + yDiversionAngle;
        }
        else if (QRname.find("W01.02") == 0) {     // Code W01.02
            FinalDronePosition.x = (QRWallCode[7].x - DronePosition.y);
            FinalDronePosition.y = (QRWallCode[7].y + DronePosition.x);
            FinalDronePosition.heading = 90 + yDiversionAngle;
        }
        else if (QRname.find("W01.03") == 0) {     // Code W01.03
            FinalDronePosition.x = (QRWallCode[8].x - DronePosition.y);
            FinalDronePosition.y = (QRWallCode[8].y + DronePosition.x);
            FinalDronePosition.heading = 90 + yDiversionAngle;
        }
        else if (QRname.find("W01.04") == 0) {     // Code W01.04
            FinalDronePosition.x = (QRWallCode[9].x - DronePosition.y);
            FinalDronePosition.y = (QRWallCode[9].y + DronePosition.x);
            FinalDronePosition.heading = 90 + yDiversionAngle;
        }
    }

    else if (QRname.find("W02.") == 0) {       // Wall 2 has been found
        if (QRname.find("W02.00") == 0) {     // Code W02.00
            FinalDronePosition.x = (QRWallCode[10].x + DronePosition.x);
            FinalDronePosition.y = (QRWallCode[10].y + DronePosition.y);
            FinalDronePosition.heading = 180 + yDiversionAngle;
        }
        else if (QRname.find("W02.01") == 0) {     // Code W02.01
            FinalDronePosition.x = (QRWallCode[11].x + DronePosition.x);
            FinalDronePosition.y = (QRWallCode[11].y + DronePosition.y);
            FinalDronePosition.heading = 180 + yDiversionAngle;
        }
        else if (QRname.find("W02.02") == 0) {     // Code W02.02
            FinalDronePosition.x = (QRWallCode[12].x + DronePosition.x);
            FinalDronePosition.y = (QRWallCode[12].y + DronePosition.y);
            FinalDronePosition.heading = 180 + yDiversionAngle;
        }
        else if (QRname.find("W02.03") == 0) {     // Code W02.03
            FinalDronePosition.x = (QRWallCode[13].x + DronePosition.x);
            FinalDronePosition.y = (QRWallCode[13].y + DronePosition.y);
            FinalDronePosition.heading = 180 + yDiversionAngle;
        }
        else if (QRname.find("W02.04") == 0) {     // Code W02.04
            FinalDronePosition.x = (QRWallCode[14].x + DronePosition.x);
            FinalDronePosition.y = (QRWallCode[14].y + DronePosition.y);
            FinalDronePosition.heading = 180 + yDiversionAngle;
        }
    }

    else if (QRname.find("W03.") == 0) {       // Wall 3 has been found
        if (QRname.find("W03.00") == 0) {     // Code W03.00
            FinalDronePosition.x = (QRWallCode[15].x + DronePosition.y);
            FinalDronePosition.y = (QRWallCode[15].y - DronePosition.x);
            FinalDronePosition.heading = 270 + yDiversionAngle;
        }
        else if (QRname.find("W03.01") == 0) {     // Code W03.01
            FinalDronePosition.x = (QRWallCode[16].x + DronePosition.y);
            FinalDronePosition.y = (QRWallCode[16].y - DronePosition.x);
            FinalDronePosition.heading = 270 + yDiversionAngle;
        }
        else if (QRname.find("W03.02") == 0) {     // Code W03.02
            FinalDronePosition.x = (QRWallCode[17].x + DronePosition.y);
            FinalDronePosition.y = (QRWallCode[17].y - DronePosition.x);
            FinalDronePosition.heading = 270 + yDiversionAngle;
        }
        else if (QRname.find("W03.03") == 0) {     // Code W03.03
            FinalDronePosition.x = (QRWallCode[18].x + DronePosition.y);
            FinalDronePosition.y = (QRWallCode[18].y - DronePosition.x);
            FinalDronePosition.heading = 270 + yDiversionAngle;
        }
        else if (QRname.find("W03.04") == 0) {     // Code W03.04
            FinalDronePosition.x = (QRWallCode[19].x + DronePosition.y);
            FinalDronePosition.y = (QRWallCode[19].y - DronePosition.x);
            FinalDronePosition.heading = 270 + yDiversionAngle;
        }
    }

}

double QR::calculateDistanceToQR(int pixel) {
    if (pixel < 38) return -1;       // If distance is more than 2,9m
    else if (pixel > 130) return -2;    // If distance is less than 1m.
    else return distanceToQR[pixel]; // Returns distance in cm.
}

void QR::initializeQR() {
    string value;
    int i = 0;
    ifstream file("../workspaces/dronemis_ws/src/dronemis/src/OpenCv/WallCoordinates.csv");
    if (!file.good()) {
        ROS_INFO("Unable to read QR csv file");
        return;
    }
    getline(file, value);
    while (!file.eof()) {
        getline(file, value, ';');
        QRWallCode[i].name = value;
        getline(file, value, ';');
        QRWallCode[i].x = std::atoi(value.c_str());
        getline(file, value);
        QRWallCode[i].y = std::atoi(value.c_str());
        i++;
    }

    ROS_INFO("%d QR codes initialized", i);

    distanceToQR[38] = 310;
    distanceToQR[39] = 298;
    distanceToQR[40] = 290;
    distanceToQR[41] = 284;
    distanceToQR[42] = 279;
    distanceToQR[43] = 274;
    distanceToQR[44] = 269;
    distanceToQR[45] = 264;
    distanceToQR[46] = 259;
    distanceToQR[47] = 254;
    distanceToQR[48] = 250;
    distanceToQR[49] = 245;
    distanceToQR[50] = 241;
    distanceToQR[51] = 236;
    distanceToQR[52] = 232;
    distanceToQR[53] = 228;
    distanceToQR[54] = 224;
    distanceToQR[55] = 220;
    distanceToQR[56] = 217;
    distanceToQR[57] = 213;
    distanceToQR[58] = 209;
    distanceToQR[59] = 206;
    distanceToQR[60] = 203;
    distanceToQR[61] = 199;
    distanceToQR[62] = 196;
    distanceToQR[63] = 193;
    distanceToQR[64] = 190;
    distanceToQR[65] = 187;
    distanceToQR[66] = 184;
    distanceToQR[67] = 181;
    distanceToQR[68] = 179;
    distanceToQR[69] = 176;
    distanceToQR[70] = 173;
    distanceToQR[71] = 171;
    distanceToQR[72] = 168;
    distanceToQR[73] = 166;
    distanceToQR[74] = 164;
    distanceToQR[75] = 161;
    distanceToQR[76] = 159;
    distanceToQR[77] = 157;
    distanceToQR[78] = 155;
    distanceToQR[79] = 153;
    distanceToQR[80] = 151;
    distanceToQR[81] = 149;
    distanceToQR[82] = 148;
    distanceToQR[83] = 146;
    distanceToQR[84] = 144;
    distanceToQR[85] = 142;
    distanceToQR[86] = 141;
    distanceToQR[87] = 139;
    distanceToQR[88] = 137;
    distanceToQR[89] = 136;
    distanceToQR[90] = 136;
    distanceToQR[91] = 133;
    distanceToQR[92] = 132;
    distanceToQR[93] = 130;
    distanceToQR[94] = 129;
    distanceToQR[95] = 128;
    distanceToQR[96] = 126;
    distanceToQR[97] = 125;
    distanceToQR[98] = 124;
    distanceToQR[99] = 122;
    distanceToQR[100] = 121;
    distanceToQR[101] = 120;
    distanceToQR[102] = 119;
    distanceToQR[103] = 118;
    distanceToQR[104] = 116;
    distanceToQR[105] = 115;
    distanceToQR[106] = 114;
    distanceToQR[107] = 113;
    distanceToQR[108] = 112;
    distanceToQR[109] = 111;
    distanceToQR[110] = 110;
    distanceToQR[111] = 108;
    distanceToQR[112] = 108;
    distanceToQR[113] = 106;
    distanceToQR[114] = 105;
    distanceToQR[115] = 104;
    distanceToQR[116] = 103;
    distanceToQR[117] = 101;
    distanceToQR[118] = 100;
    distanceToQR[119] = 99;
    distanceToQR[120] = 98;
    distanceToQR[121] = 97;
    distanceToQR[122] = 96;
    distanceToQR[123] = 95;
    distanceToQR[124] = 94;
    distanceToQR[125] = 93;
    distanceToQR[126] = 92;
    distanceToQR[127] = 91;
    distanceToQR[128] = 90;
    distanceToQR[129] = 89;
    distanceToQR[130] = 88;
}


