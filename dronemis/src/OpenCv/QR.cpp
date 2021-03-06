#include "QR.h"

using namespace cv;
using namespace std;
using namespace zbar;

QR::QR(CV_Handler *cv) {
    initializeQR();
    cvHandler = cv;
}

#define FRAME_COUNT 1

DronePos QR::checkQR(void) {
#ifdef DEBUG
    ROS_INFO("inside the checkQR");
#endif
    RoomDronePosition.resetCoordinates();
    DronePosition.resetCoordinates();
    yRatioTemp = 0;
    int frameCount = 0;
    int averageCount = 0;
    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    while (frameCount++ != FRAME_COUNT) {

        while (!cvHandler->imageReady);

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
        Image image((unsigned int)width, (unsigned int)height, "Y800", raw, (unsigned long)width * height);


        // scan the image for barcodes
        int numberQR = scanner.scan(image);                                         // Returns number of codes in the image.
        if (numberQR == 1)
            averageCount++;

        int x0, x1, x2, x3, y0, y1, y2, y3, xsize, ysize, xmidt, QRsize;
        vector<Point> vp;

        // extract results

        for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {


            // do something useful with results
            int n = symbol->get_location_size();        // Returns 4 if QR code is scanned
            for (int i = 0; i < n; i++) {
                vp.push_back(Point(symbol->get_location_x((unsigned int)i), symbol->get_location_y((unsigned int)i)));      //Update VP
             }

            x0 = symbol->get_location_x(0);    //
            x1 = symbol->get_location_x(1);    //
            x2 = symbol->get_location_x(2);    //
            x3 = symbol->get_location_x(3);    //  Saves the size of the QR code being detected
            y0 = symbol->get_location_y(0);    //
            y1 = symbol->get_location_y(1);    //
            y2 = symbol->get_location_y(2);    //
            y3 = symbol->get_location_y(3);    //
            xsize = (abs((x0 - x2)) + abs((x1 - x3))) / 2;                          // Calculate Vertical size of the QR
            ysize = (abs((y0 - y2)) + abs((y3 - y1))) / 2;                          // Calculate Horizontal size of the QR
            xmidt = (x0 + x1 + x2 + x3) / 4;                                        // The horizontal middle of the QR

            QRsize = (xsize + ysize) / 2;                                           // The size of the QR
            double distancetoQRdouble = calculateDistanceToQR(QRsize);               // To prevent errors when typecasting
            int distancetoQR = (int)distancetoQRdouble; // double to int
            std::string QRName = symbol->get_data();

            double yleft, yright, ytemp, yratio;

            yleft = y1 - y0;                                                        // calculate left side of QR
            yright = y2 - y3;                                                       // Calculate right side of QR

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


#ifdef DEBUG_QR_COUT                                                                // Debugging information
            //cout << "yleft / yright = " << yleft << " / " << yright << endl;
            //cout << "averageCount = " << averageCount << " and yratio = " << yratio << endl;

            //cout << "yratio = " << yratio << endl;
            //cout << "yratioTemp = " << yRatioTemp << endl;
#endif

            y1Diversion = (yratio * 360.0395) - 359.2821;                           // Two different equations that sum up the angle of the QR
            y2Diversion = (yratio * 637.3656) - 642.2072;                           //

#ifdef DEBUG_QR_COUT
            cout << "distanceToQR = " << distancetoQR << "cm, with the text: " << QRName << endl << endl;
            //cout << "y1Diversion (Parallel) = " << y1Diversion << endl;
            //cout << "y2Diversion (Kig på QR)= " << y2Diversion << endl;
            //cout << "yratio = " << yratio << endl;
#endif

            double xDistanceStatic = 3.61194;                                       // Calculated using 243 pixels for 67cm on 150cm distance.
            double xDistancedouble = ((xmidt - 320) / xDistanceStatic * distancetoQR / 150); // Calculate in cm's where the center of the camera is, according to the center of the QR
            int xDistance = (int) xDistancedouble;

#ifdef DEBUG_QR_COUT
            cout << "Kamera center er: " << xDistance << "cm til venstre for QR-koden" << endl;


#endif
            double yDiversionAngledouble;
            if (yratio < 1.06) yDiversionAngledouble = (y1Diversion + y2Diversion) / 2 * direction;
            else yDiversionAngledouble = y2Diversion * direction;
            yDiversionAngle = (int)yDiversionAngledouble;


            DronePosition.relativeX = (distancetoQR/1.5 * std::sin(yDiversionAngle * (M_PI / 180)))*1.2; // xDistance (Forskydning)
            DronePosition.relativeY = (distancetoQR * std::cos(yDiversionAngle * (M_PI / 180)));
            //cout << "yDiversionAngle = " << abs(yDiversionAngle) << endl;
            bool positionLock;
            if (abs(yDiversionAngle) < 25) positionLock = 1;
            else positionLock = 0;
            //cout << "positionLock = " << positionLock << endl;


            //**************************************
            //**** Match up with QR coordinates ****
            //**************************************

            calculateRoomDronePostition(QRName, DronePosition.relativeX, DronePosition.relativeY, positionLock,
                                        averageCount,
                                        xDistance, yDiversionAngle);

#ifdef DEBUG_QR_COUT
            //ROS_INFO("TESTMODE ACTIVE");
            cout << "Droneposition relative (x,y) = " << DronePosition.relativeX << "," << DronePosition.relativeY <<
            endl;
            cout << "Angle = " << yDiversionAngle << endl;
            cout << "PositionLock = " << RoomDronePosition.positionLocked << " on QR code: " << QRName << endl << endl;

            //cout << "RoomAngle relative to QR =" << RoomDronePosition.angle << endl;
            //cout << symbol->get_data() << endl;
            //cout << "RoomDroneposition(x,y) = " << RoomDronePosition.x << "," << RoomDronePosition.y << endl;
            //cout << "RoomDroneHeading = " << RoomDronePosition.heading << endl;
#endif
        }
    }
    return RoomDronePosition;
}



void QR::calculateRoomDronePostition(std::string QRname, int relativeX, int relativeY, bool positionLocked,
                                     int numberOfQRs, double cameraPointing, double angle) {
    RoomDronePosition.relativeX = relativeX;
    RoomDronePosition.relativeY = relativeY;
    RoomDronePosition.positionLocked = positionLocked;
    RoomDronePosition.numberOfQRs = numberOfQRs;
    RoomDronePosition.cameraPointing = cameraPointing;
    RoomDronePosition.angle = angle;
    if (QRname.find("W00") == 0) {       // Wall 0 has been found
        RoomDronePosition.wallNumber = 0;
        if (QRname.find("W00.00") == 0) {     // Code W00.00
            RoomDronePosition.x = (QRWallCode[0].x - relativeX);
            RoomDronePosition.y = (QRWallCode[0].y - relativeY);
            RoomDronePosition.heading = yDiversionAngle;

        }
        else if (QRname.find("W00.01") == 0) {     // Code W00.01
            RoomDronePosition.x = (QRWallCode[1].x - relativeX);
            RoomDronePosition.y = (QRWallCode[1].y - relativeY);
            RoomDronePosition.heading = yDiversionAngle;
        }
        else if (QRname.find("W00.02") == 0) {     // Code W00.02
            RoomDronePosition.x = (QRWallCode[2].x - relativeX);
            RoomDronePosition.y = (QRWallCode[2].y - relativeY);
            RoomDronePosition.heading = yDiversionAngle;
        }
        else if (QRname.find("W00.03") == 0) {     // Code W00.03
            RoomDronePosition.x = (QRWallCode[3].x - relativeX);
            RoomDronePosition.y = (QRWallCode[3].y - relativeY);
            RoomDronePosition.heading = yDiversionAngle;
        }
        else if (QRname.find("W00.04") == 0) {     // Code W00.04
            RoomDronePosition.x = (QRWallCode[4].x - relativeX);
            RoomDronePosition.y = (QRWallCode[4].y - relativeY);
            RoomDronePosition.heading = yDiversionAngle;
        }
    }

    else if (QRname.find("W01.") == 0) {       // Wall 1 has been found
        RoomDronePosition.wallNumber = 1;
        if (QRname.find("W01.00") == 0) {     // Code W01.00
            RoomDronePosition.x = (QRWallCode[5].x - relativeY);
            RoomDronePosition.y = (QRWallCode[5].y + relativeX);
            RoomDronePosition.heading = 90 + yDiversionAngle;
        }
        else if (QRname.find("W01.01") == 0) {     // Code W01.01
            RoomDronePosition.x = (QRWallCode[6].x - relativeY);
            RoomDronePosition.y = (QRWallCode[6].y + relativeX);
            RoomDronePosition.heading = 90 + yDiversionAngle;
        }
        else if (QRname.find("W01.02") == 0) {     // Code W01.02
            RoomDronePosition.x = (QRWallCode[7].x - relativeY);
            RoomDronePosition.y = (QRWallCode[7].y + relativeX);
            RoomDronePosition.heading = 90 + yDiversionAngle;
        }
        else if (QRname.find("W01.03") == 0) {     // Code W01.03
            RoomDronePosition.x = (QRWallCode[8].x - relativeY);
            RoomDronePosition.y = (QRWallCode[8].y + relativeX);
            RoomDronePosition.heading = 90 + yDiversionAngle;
        }
        else if (QRname.find("W01.04") == 0) {     // Code W01.04
            RoomDronePosition.x = (QRWallCode[9].x - relativeY);
            RoomDronePosition.y = (QRWallCode[9].y + relativeX);
            RoomDronePosition.heading = 90 + yDiversionAngle;
        }
    }

    else if (QRname.find("W02.") == 0) {       // Wall 2 has been found
        RoomDronePosition.wallNumber = 2;
        if (QRname.find("W02.00") == 0) {     // Code W02.00
            RoomDronePosition.x = (QRWallCode[10].x + relativeX);
            RoomDronePosition.y = (QRWallCode[10].y + relativeY);
            RoomDronePosition.heading = 180 + yDiversionAngle;
        }
        else if (QRname.find("W02.01") == 0) {     // Code W02.01
            RoomDronePosition.x = (QRWallCode[11].x + relativeX);
            RoomDronePosition.y = (QRWallCode[11].y + relativeY);
            RoomDronePosition.heading = 180 + yDiversionAngle;
        }
        else if (QRname.find("W02.02") == 0) {     // Code W02.02
            RoomDronePosition.x = (QRWallCode[12].x + relativeX);
            RoomDronePosition.y = (QRWallCode[12].y + relativeY);
            RoomDronePosition.heading = 180 + yDiversionAngle;
        }
        else if (QRname.find("W02.03") == 0) {     // Code W02.03
            RoomDronePosition.x = (QRWallCode[13].x + relativeX);
            RoomDronePosition.y = (QRWallCode[13].y + relativeY);
            RoomDronePosition.heading = 180 + yDiversionAngle;
        }
        else if (QRname.find("W02.04") == 0) {     // Code W02.04
            RoomDronePosition.x = (QRWallCode[14].x + relativeX);
            RoomDronePosition.y = (QRWallCode[14].y + relativeY);
            RoomDronePosition.heading = 180 + yDiversionAngle;
        }
    }

    else if (QRname.find("W03.") == 0) {       // Wall 3 has been found
        RoomDronePosition.wallNumber = 3;
        if (QRname.find("W03.00") == 0) {     // Code W03.00
            RoomDronePosition.x = (QRWallCode[15].x + relativeY);
            RoomDronePosition.y = (QRWallCode[15].y - relativeX);
            RoomDronePosition.heading = 270 + yDiversionAngle;
        }
        else if (QRname.find("W03.01") == 0) {     // Code W03.01
            RoomDronePosition.x = (QRWallCode[16].x + relativeY);
            RoomDronePosition.y = (QRWallCode[16].y - relativeX);
            RoomDronePosition.heading = 270 + yDiversionAngle;
        }
        else if (QRname.find("W03.02") == 0) {     // Code W03.02
            RoomDronePosition.x = (QRWallCode[17].x + relativeY);
            RoomDronePosition.y = (QRWallCode[17].y - relativeX);
            RoomDronePosition.heading = 270 + yDiversionAngle;
        }
        else if (QRname.find("W03.03") == 0) {     // Code W03.03
            RoomDronePosition.x = (QRWallCode[18].x + relativeY);
            RoomDronePosition.y = (QRWallCode[18].y - relativeX);
            RoomDronePosition.heading = 270 + yDiversionAngle;
        }
        else if (QRname.find("W03.04") == 0) {     // Code W03.04
            RoomDronePosition.x = (QRWallCode[19].x + relativeY);
            RoomDronePosition.y = (QRWallCode[19].y - relativeX);
            RoomDronePosition.heading = 270 + yDiversionAngle;
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
    distanceToQR[99] = 122+1;
    distanceToQR[100] = 121+1;
    distanceToQR[101] = 120+1;
    distanceToQR[102] = 119+2;
    distanceToQR[103] = 118+2;
    distanceToQR[104] = 116+2;
    distanceToQR[105] = 115+3;
    distanceToQR[106] = 114+3;
    distanceToQR[107] = 113+3;
    distanceToQR[108] = 112+4;
    distanceToQR[109] = 111+4;
    distanceToQR[110] = 110+4;
    distanceToQR[111] = 108+5;
    distanceToQR[112] = 108+5;
    distanceToQR[113] = 106+5;
    distanceToQR[114] = 105+6;
    distanceToQR[115] = 104+6;
    distanceToQR[116] = 103+6;
    distanceToQR[117] = 101+7;
    distanceToQR[118] = 100+7;
    distanceToQR[119] = 99+7;
    distanceToQR[120] = 98+8;
    distanceToQR[121] = 97+8;
    distanceToQR[122] = 96+8;
    distanceToQR[123] = 95+9;
    distanceToQR[124] = 94+9;
    distanceToQR[125] = 93+9;
    distanceToQR[126] = 92+9;
    distanceToQR[127] = 91+10;
    distanceToQR[128] = 90+10;
    distanceToQR[129] = 89+10;
    distanceToQR[130] = 88+10;
}


