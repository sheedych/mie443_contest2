#include <imagePipeline.h>
#include <math.h>
#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"
using namespace cv;
using namespace cv::xfeatures2d;

ImagePipeline::ImagePipeline(ros::NodeHandle &n)
{
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        if (isValid)
        {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }
}

int ImagePipeline::getTemplateID(Boxes &boxes, LaserData laserData)
{
    int template_id = 0;
    if (!isValid)
    {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    }
    else if (img.empty() || img.rows <= 0 || img.cols <= 0)
    {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    }
    else
    {
        /***YOUR CODE HERE***/
        // Use: boxes.templates

        //vector stuff here copy stuff in
        int minHessian = 400;
        Mat img_scene = img;
        double global_min_dist = 100;
        double global_max_dist = 0;
        double min_dist_threshold = 0.1;
        const float ratio = 0.7f;

        for (std::vector<Mat>::iterator it = boxes.templates.begin(); it != boxes.templates.end(); ++it)
        {
            std::cout << "iterating" << std::endl;
            //grab the image object template thing
            Mat img_object = *it;

            //if i cannot read the image, freak out
            if (!img_object.data || !img_scene.data)
                std::cout << "cant read" << std::endl;
            Ptr<SURF> detector = SURF::create(minHessian);

            std::vector<KeyPoint> keypoints_object, keypoints_scene;
            Mat descriptors_object, descriptors_scene;

            detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);
            detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);

            double max_dist = 0;
            double min_dist = 100;
            int besti = 0;

            //FlannBasedMatcher matcher;
            BFMatcher matcher;
            std::vector<std::vector<DMatch>> matches;
            //matcher.match(descriptors_object, descriptors_scene, matches);
            matcher.knnMatch(descriptors_object, descriptors_scene, matches, 2);

            std::vector<DMatch> good_matches;
            for (int i = 0; i < matches.size(); i++)
            {
                if (matches[i][0].distance < ratio * matches[i][1].distance)
                {
                    good_matches.push_back(matches[i][0]);
                }
            }

            int num_good_matches = good_matches.size();

            Mat img_matches;
            drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

            std::vector<Point2f> obj;
            std::vector<Point2f> scene;

            for (int i = 0; i < good_matches.size(); i++)
            {

                obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
                scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
            }

            //put an if statement around this making sure obj and scene contain at least 4 things
            try
            {
                //if(obj.size() >= 4 && scene.size() >= 4) {
                // previously 70, need to find a balance between detecting the cinamon toast crunch image and detecting something random when nothing is there.
                if (num_good_matches >= 45)
                {
                    Mat H = findHomography(obj, scene, RANSAC);

                    //--Get the corners from the image_1 (the object to be "detected")
                    std::vector<Point2f> obj_corners(4);
                    obj_corners[0] = cvPoint(0, 0);
                    obj_corners[1] = cvPoint(img_object.cols, 0);
                    obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
                    obj_corners[3] = cvPoint(0, img_object.rows);
                    std::vector<Point2f> scene_corners(4);

                    if (!H.empty())
                    {
                        perspectiveTransform(obj_corners, scene_corners, H);
                    }

                    float perimeter = 0.0;
                    for (int i = 0; i < 4; i++)
                    {
                        std::cout << i << "\n";
                        float euc;
                        if (i == 3)
                        {
                            euc = sqrt(pow((scene_corners[i].x - scene_corners[0].x), 2) + pow((scene_corners[i].y - scene_corners[0].y), 2));
                        }
                        else
                        {
                            euc = sqrt(pow((scene_corners[i].x - scene_corners[i+1].x), 2) + pow((scene_corners[i].y - scene_corners[i+1].y), 2));
                        }
                        std::cout << "Euc: " << euc << "\n";
                        perimeter = perimeter + euc;
                    }

                    std::cout << "perimeter = " << perimeter << "\n";

                    if (perimeter < 500)
                    {
                        std::cout << "The perimeter of the detected object is too small";
                        continue;
                    }

                    //-- Draw lines between the corners (the mapped object in the scene - image_2)
                    line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
                    line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
                    line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
                    line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);

                    std::cout << "Displaying image" << template_id << "  now" << std::endl;

                    cv::imshow("view", img_matches);
                    cv::waitKey(10);
                    return template_id;
                }
                else
                {
                    std::cout << "The object or scene was not big enough" << std::endl;
                }
            }
            catch (int e)
            {
                return -1;
            }
            template_id++;
        }
    }
    return -1;
}
