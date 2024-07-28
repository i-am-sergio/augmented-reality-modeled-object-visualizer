#include "LoadCamera.hpp"

float markerLength = 0.1f;
float dx = -0.05f, dy = -0.05f, dz = 0.15f;

std::vector<cv::Point3f> LoadCamera::getCubeVertices()
{
    return {
        {dx, dy, dz}, {dx, dy + markerLength, dz}, {dx + markerLength, dy + markerLength, dz}, {dx + markerLength, dy, dz}, {dx, dy, dz - markerLength}, {dx, dy + markerLength, dz - markerLength}, {dx + markerLength, dy + markerLength, dz - markerLength}, {dx + markerLength, dy, dz - markerLength}};
}

std::vector<cv::Point3f> LoadCamera::getPrismVertices()
{
    return {
        {dx, dy, dz}, {dx, dy + markerLength, dz}, {dx + markerLength, dy + markerLength, dz}, {dx + markerLength, dy, dz}, {dx + 0.5f * markerLength, dy + 0.5f * markerLength, dz - markerLength}, {dx + 0.5f * markerLength, dy + 0.5f * markerLength, dz - markerLength}, {dx + 1.5f * markerLength, dy + 1.5f * markerLength, dz - markerLength}, {dx + 1.5f * markerLength, dy + 1.5f * markerLength, dz - markerLength}};
}

void LoadCamera::drawShape(cv::Mat &frame, const std::vector<cv::Point2f> &imgpts)
{
    if (imgpts.size() != 8) // Ensure imgpts size is correct
        return;
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(std::vector<cv::Point>(imgpts.begin(), imgpts.begin() + 4));
    contours.push_back(std::vector<cv::Point>(imgpts.begin() + 4, imgpts.end()));
    cv::drawContours(frame, contours, 0, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    for (int i = 0; i < 4; ++i)
        cv::line(frame, imgpts[i], imgpts[i + 4], cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    cv::drawContours(frame, contours, 1, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
}

LoadCamera::LoadCamera(vector<ObjectProjection> &objectsProjections)
{
    // Objects projections
    this->objectsProjections = objectsProjections;
    // Default camera calibration parameters (needs to be calibrated for actual use)
    this->cameraMatrix = (cv::Mat_<float>(3, 3) << 800, 0, 320, 0, 800, 240, 0, 0, 1);
    this->distCoeffs = cv::Mat::zeros(5, 1, CV_32F);
    this->arucoDict = new cv::aruco::Dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    this->detectorParams = new cv::aruco::DetectorParameters();
    this->markerLength = 0.1f;
    this->rotation = cv::Vec3d(0, 0, 0); // Default rotation
    this->animationConfig.iluminate = false;
    this->animationConfig.rotate = true;
    this->animationConfig.iluminate = false;
    this->animationConfig.scaleObject = 0.0;
    this->animationConfig.xTranslation = 0.0;
    this->animationConfig.yTranslation = 0.0;
    this->animationConfig.step = 0.01;
    this->animationConfig.scaleStep = 0.1;
    this->animationConfig.rotationSpeedZ = 0;
    this->animationConfig.baseColor = cv::Scalar(0, 255, 0); // Green
}

LoadCamera::~LoadCamera()
{
    cap.release();
}

bool LoadCamera::openCamera(int cameraIndex)
{
    cap.open(cameraIndex);
    return cap.isOpened();
}

void LoadCamera::onMouse(int event, int x, int y, int, void *userdata)
{
    LoadCamera *self = static_cast<LoadCamera *>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        self->isDragging = true;
        self->lastMousePos = cv::Point(x, y);
    }
    else if (event == cv::EVENT_MOUSEMOVE && self->isDragging)
    {
        int dx = x - self->lastMousePos.x;
        int dy = y - self->lastMousePos.y;
        self->lastMousePos = cv::Point(x, y);
        self->rotation[0] += dy * 0.01; // Rotate around x-axis
        self->rotation[1] += dx * 0.01; // Rotate around y-axis
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
        self->isDragging = false;
    }
}

void LoadCamera::showCamera()
{
    cv::namedWindow("Camera");
    cv::setMouseCallback("Camera", onMouse, this);

    while (true)
    {
        this->cap >> frame;
        if (frame.empty())
            break;

        // Convert frame to grayscale
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Detect markers
        std::vector<std::vector<cv::Point2f>> markerCorners;
        std::vector<int> markerIDs;
        std::vector<std::vector<cv::Point2f>> reject;

        cv::aruco::detectMarkers(gray, arucoDict, markerCorners, markerIDs, detectorParams, reject);

        if (!markerIDs.empty())
        {
            // Draw detected markers and display their IDs
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIDs);

            // Estimate pose of each marker
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

            for (size_t i = 0; i < markerIDs.size(); i++)
            {
                // Draw axis for each marker
                cv::drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength);

                // Select the vertices based on the marker ID
                if (markerIDs[i] == 11)
                {
                    // objectsProjections[0].drawObject(frame, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, movement);
                    objectsProjections[0].drawObject(frame, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, rotation, animationConfig);
                }
                else if (markerIDs[i] == 12)
                {
                    // objectsProjections[1].drawObject(frame, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, movement);
                    objectsProjections[1].drawObject(frame, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, rotation, animationConfig);
                }
                else if (markerIDs[i] == 13)
                {
                    // objectsProjections[2].drawObject(frame, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, movement);
                    objectsProjections[2].drawObject(frame, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, rotation, animationConfig);
                }
                else if (markerIDs[i] == 14)
                {
                    // objectsProjections[4].drawObject(frame, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, movement);
                    objectsProjections[4].drawObject(frame, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, rotation, animationConfig);
                }
                else if (markerIDs[i] == 40)
                {
                    auto vertices = getCubeVertices();
                    std::vector<cv::Point2f> imgpts;
                    cv::projectPoints(vertices, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imgpts);
                    drawShape(frame, imgpts);
                }
                else if (markerIDs[i] == 41)
                {
                    auto vertices = getPrismVertices();
                    std::vector<cv::Point2f> imgpts;
                    cv::projectPoints(vertices, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imgpts);
                    drawShape(frame, imgpts);
                }
            }
        }

        cv::imshow("Camera", frame);
        int key = cv::waitKey(10);

        rotation[2] += animationConfig.rotationSpeedZ;
        if (rotation[2] > CV_PI)
            rotation[2] -= 2 * CV_PI;

        // stop to press 'q' or 'esc'
        if (key == 27 || key == 'q')
        {
            break;
        }
        if (cv::getWindowProperty("Camera", cv::WND_PROP_AUTOSIZE) == -1)
        {
            break;
        }
        if (key == 'c')
        {
            currentColorIndex = (currentColorIndex + 1) % baseColors.size();
            this->animationConfig.baseColor = baseColors[currentColorIndex];
        }
        if (key == 'g')
        {
            this->animationConfig.rotate = false;
            this->animationConfig.iluminate = false;
            this->animationConfig.spin = true;
        }
        if (key == 'i')
        {
            this->animationConfig.rotate = false;
            this->animationConfig.iluminate = true;
            this->animationConfig.spin = false;
        }
        if (key == 'r')
        {
            this->animationConfig.rotate = true;
            this->animationConfig.iluminate = false;
            this->animationConfig.spin = false;
            this->animationConfig.rotationSpeedZ = 0.1;
        }

        if (key == 'z')
        {
            this->animationConfig.spin = true;
            this->animationConfig.rotate = false;
            this->animationConfig.rotationSpeedZ = 0;
        }
        if (key == 'd')
        {
            this->animationConfig.xTranslation += this->animationConfig.step;
        }
        if (key == 'a')
        {
            this->animationConfig.xTranslation -= this->animationConfig.step;
        }
        if (key == 'w')
        {
            animationConfig.yTranslation -= this->animationConfig.step;
        }
        if (key == 's')
        {
            this->animationConfig.yTranslation += this->animationConfig.step;
        }
        if (key == 'k')
        {
            this->animationConfig.scaleObject += this->animationConfig.scaleStep;
        }
        if (key == 'm')
        {
            this->animationConfig.scaleObject -= this->animationConfig.scaleStep;
        }
    }
}