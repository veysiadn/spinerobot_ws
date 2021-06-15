#ifndef VIDEO_CAPTURE_HPP
#define VIDEO_CAPTURE_HPP

#include <QPixmap>
#include <QImage>
#include <QThread>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>

#define ID_CAMERA 2

class VideoCapture : public QThread
{
    Q_OBJECT
public:
    VideoCapture(QObject *parent = nullptr);
    QPixmap pixmap() const
    {
        return pixmap_cap;
    }
signals:
    void NewPixmapCapture(); //capture a frame
protected:
    void run() override;
private:

    QPixmap pixmap_cap;              //Qt image
    cv::Mat frame_cap;               //OpenCV image
    cv::VideoCapture video_cap;   //video capture

    unsigned long frame_rate = 30;

    QImage cvMatToQImage(const cv::Mat &inMat);
    QPixmap cvMatToQPixmap(const cv::Mat &inMat );
};

#endif // MYVIDEOCAPTURE_H
