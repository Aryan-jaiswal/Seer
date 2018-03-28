#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"

using namespace std;
using namespace cv;

Stitcher::Mode mode = Stitcher::PANORAMA;

vector<Mat> imgs;
int main(int argc ,char* argv[])
{ int i=0,j=0;
	const string vid = argv[1]
    VideoCapture pika(vid);
    Mat img1;
    while(pika.isopened())
    {   
        img1>>pika;
    	if(i%20==0)
    	{
    		imwrite(img1,imgs[j]);
    		j++;
    	}

        i++;

    }
	mat panorama;

    Ptr<Stitcher> stitcher = Stitcher::create(mode, true);
    Stitcher::Status status = stitcher->stitch(imgs, panorama);

 if (status != Stitcher::OK)
    {
        cout << "Can't stitch images\n";
        return -1;
    }

    imwrite("result.jpg", pano);
     
    imshow("hello", pano);
     
    waitKey(0);
    return 0;

}
