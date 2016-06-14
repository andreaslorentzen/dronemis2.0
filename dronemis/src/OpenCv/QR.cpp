#include "QR.h"

using namespace cv;
using namespace std;
using namespace zbar;

QR::QR(CV_Handler *cv) {
    initializeQR();
    cvHandler = cv;
}

int QR::checkQR(void) {

    cv::Mat img(cvHandler->storedImageBW.size().y,
                  cvHandler->storedImageBW.size().x,
                    CV_8UC1,
                  cvHandler->storedImageBW.data());

    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    {
        Mat grey;

        cvtColor(img,grey,CV_BGR2GRAY);
        int width = img.cols;
        int height = img.rows;
        uchar *raw = grey.data;

        // wrap image data
        Image image(width, height, "Y800", raw, width * height);


        // scan the image for barcodes
        int n = scanner.scan(image);                    // Returns number of codes in the image.
        //cout << "Number of QR codes in the image is " << n << endl;
        int x0[5], x1[5], x2[5], x3[5], y0[5], y1[5], y2[5], y3[5], xsize[5], ysize[5], xmidt[5], ymidt[5], QRsize[5];
        int QRcount = 0;    // Used to seperate the found QR codes
        double QRdistance[5];


        // extract results
        for(Image::SymbolIterator symbol = image.symbol_begin()    ; symbol != image.symbol_end()     ; ++symbol) {
            vector<Point> vp;

            // do something useful with results
            //cout << "decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' << " " << endl;
            int n = symbol->get_location_size();        // Returns 4 if QR code is scanned
            for (int i = 0; i < n; i++) {
                vp.push_back(Point(symbol->get_location_x(i), symbol->get_location_y(i)));      //Update VP
                //cout << "symbol->get_location_x = " << symbol->get_location_x(i) << endl;
                //cout << "symbol->get_location_y = " << symbol->get_location_y(i) << endl;
            }

            //cout << "vp = " << vp.at(1) << endl;
            //cout << "myvector contains:";
            //½cout << "VectorPoint contains " << vp << endl;
            //std::cout << '\n';
            x0[QRcount] = symbol->get_location_x(0);    //
            x1[QRcount] = symbol->get_location_x(1);    //
            x2[QRcount] = symbol->get_location_x(2);    //
            x3[QRcount] = symbol->get_location_x(3);    //  Saves the size of the QR code being detected
            y0[QRcount] = symbol->get_location_y(0);    //
            y1[QRcount] = symbol->get_location_y(1);    //
            y2[QRcount] = symbol->get_location_y(2);    //
            y3[QRcount] = symbol->get_location_y(3);    //

            xsize[QRcount] = (abs((x0[QRcount] - x2[QRcount]))+abs((x1[QRcount]-x3[QRcount])))/2;      // Vertical size of the QR
            ysize[QRcount] = (abs((y0[QRcount] - y2[QRcount]))+abs((y3[QRcount]-y1[QRcount])))/2;      // Horizontal size of the QR

            xmidt[QRcount] = (x0[QRcount]+x1[QRcount]+x2[QRcount]+x3[QRcount])/4;           // The horizontal middle of the QR
            ymidt[QRcount] = (y0[QRcount]+y1[QRcount]+y2[QRcount]+y3[QRcount])/4;           // The vertical middle of the QR

            QRsize[QRcount] = (xsize[QRcount]+ysize[QRcount])/2;                    // The size of the QR
            QRdistance[QRcount] = (-0.000003055555555555479*(QRsize[QRcount]^3))+(0.0009988095238095032*(QRsize[QRcount]^2))-(0.120253968253967*QRsize[QRcount])+6.30;

            cout << "QR[" << QRcount << "], Placering (x,y) = " << xmidt[QRcount] << "," << ymidt[QRcount] << ". Distance = " << calculateDistance(QRsize[QRcount])
            << "cm, with the text: " << symbol->get_data() << endl << endl;

            int yleft[5], yright[5];
            yleft[QRcount] = y1[QRcount] - y0[QRcount];
            yright[QRcount] = y2[QRcount] - y3[QRcount];

            //int displacementAngle[5];
            //displacementAngle[QRcount] =

            cout << "Yleft = " << yleft[QRcount] << ", Yright = " << yright[QRcount] << endl;

            //cout<<"Angle: "<<r.angle<<endl;
            QRcount++;
        }
        return 0;
        /*imshow("MyVideo", frame); //show the frame in "MyVideo" window
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            break;
        }*/
    }
}

double QR::calculateDistance(int pixel){
    if(pixel < 38) return -1;       // If distance is more than 2,9m
    else if (pixel > 130) return -2;    // If distance is less than 1m.
    else return distanceToQR[pixel]; // Returns distance in cm.
}

void QR::initializeQR() {

    QRWallCode[0].x = 188;  // 1.88m
    QRWallCode[0].y = 1055; // 10.55m
    QRWallCode[0].name = "W00.00";

    QRWallCode[1].x = 338;  // 3.38m
    QRWallCode[1].y = 1060; // 10.60m
    QRWallCode[1].name = "W00.01";

    QRWallCode[2].x = 515;  // 5.15m
    QRWallCode[2].y = 1055; // 10.55m
    QRWallCode[2].name = "W00.02";

    QRWallCode[3].x = 694;  // 6.94m
    QRWallCode[3].y = 1060; // 10.60m
    QRWallCode[3].name = "W00.03";

    QRWallCode[4].x = 840;  // 8.40m
    QRWallCode[4].y = 1055; // 10.55m
    QRWallCode[4].name = "W00.04";

    QRWallCode[5].x = 926;  // 9.26m
    QRWallCode[5].y = 904;  // 9.04m
    QRWallCode[5].name = "W01.00";

    QRWallCode[6].x = 926;  // 9.26m
    QRWallCode[6].y = 721;  // 7.21m
    QRWallCode[6].name = "W01.01";

    QRWallCode[7].x = 926;  // 9.26m
    QRWallCode[7].y = 566;  // 5.56m
    QRWallCode[7].name = "W01.02";

    QRWallCode[8].x = 926;  // 9.26m
    QRWallCode[8].y = 324;  // 3.24m
    QRWallCode[8].name = "W01.03";

    QRWallCode[9].x = 926;  // 9.26m
    QRWallCode[9].y = 115;  // 1.15m
    QRWallCode[9].name = "W01.04";

    QRWallCode[10].x = 847;  // 8.47m
    QRWallCode[10].y = -10;  // -0.10m
    QRWallCode[10].name = "W02.00";

    QRWallCode[11].x = 656;  // 6.56m
    QRWallCode[11].y = -77;  // -0.77m
    QRWallCode[11].name = "W02.01";

    QRWallCode[12].x = 514;  // 5.14m
    QRWallCode[12].y = 0;    // 0m
    QRWallCode[12].name = "W02.02";

    QRWallCode[13].x = 328;  // 3.28m
    QRWallCode[13].y = 0;    // 0m
    QRWallCode[14].name = "W02.03";

    QRWallCode[15].x = 143;  // 1.43m
    QRWallCode[15].y = 0;    // 0m
    QRWallCode[15].name = "W02.04";

    QRWallCode[16].x = 0;    // 0m
    QRWallCode[16].y = 108;  // 1.08m
    QRWallCode[16].name = "W03.00";

    QRWallCode[17].x = 0;    // 0m
    QRWallCode[17].y = 357;  // 3.57m
    QRWallCode[17].name = "W03.01";

    QRWallCode[18].x = 0;   // 0m
    QRWallCode[18].y = 561; // 5.61m
    QRWallCode[18].name = "W03.02";

    QRWallCode[19].x = 0;   // 0m
    QRWallCode[19].y = 740; // 7.40m
    QRWallCode[19].name = "W03.03";

    QRWallCode[20].x = 0;   // 0m
    QRWallCode[20].y = 997; // 9.97m
    QRWallCode[20].name = "W03.04";


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


