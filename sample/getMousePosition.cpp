
    #include "opencv/highgui.h"  
    #include "opencv/cv.h" 

    void on_mouse(int event, int x, int y, int flags, void* img)  
    {  
        CvFont font;  
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);  
        if (event == CV_EVENT_LBUTTONDOWN)  
        {  
            IplImage *timg = cvCloneImage((IplImage *)img);  
            CvPoint pt = cvPoint(x, y);  
            char temp[16];  
            sprintf(temp, "(%d,%d)", x, y);  
            cvPutText(timg, temp, pt, &font, CV_RGB(250,0,0));  
            cvCircle(timg, pt, 2, cvScalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);  
            cvShowImage("src", timg);  
            cvReleaseImage(&timg);  
        }  
    }  
      
    int main(int argc, char **argv)  
    {  
        IplImage *img = cvLoadImage(argv[1]);  
        cvNamedWindow("src", 1);  
        cvSetMouseCallback("src", on_mouse,img);  
        cvShowImage("src", img);  
        cvWaitKey(0);  
        cvDestroyAllWindows();  
        cvReleaseImage(&img);  
        return 0;  
    }  