// Copyright 2025 ZhangSuHang

#ifndef OPENVINO_TEST_OPENVINOINFER_H
#define OPENVINO_TEST_OPENVINOINFER_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <future>

//用前先根据模型修改好要修改的地方
using namespace cv;
using namespace std;

// #define mean

namespace rm_auto_aim
{



constexpr double SMALL_ARMOR_WIDTH = 135.0 / 1000.0 ; // 35mm
constexpr double SMALL_ARMOR_HEIGHT = 55.0 / 1000.0 ;// 60mm
constexpr double LARGE_ARMOR_WIDTH = 225.0 / 1000.0 ; // 230mm
constexpr double LARGE_ARMOR_HEIGHT = 55.0 / 1000.0 ;// 60mm

// 15 degree in rad
constexpr double FIFTTEN_DEGREE_RAD = 15 * CV_PI / 180;

// Armor type
enum class ArmorType { SMALL, LARGE, INVALID };
inline std::string armorTypeToString(const ArmorType &type) {
  switch (type) {
    case ArmorType::SMALL:
      return "small";
    case ArmorType::LARGE:
      return "large";
    default:
      return "invalid";
  }
}

struct ArmorObject
{
    cv::Rect_<float> rect; //
    float landmarks[8]; //4个关键点，储存xy，顺序是从左上开始，逆时针
    int label;
    float prob;
    int color;      //blue:1 , red:0
    double length;
    double width;
    double ratio;   //长宽比
    ArmorType type;
    cv::Point2f center;   //装甲板中心

    // Armor size, Unit: m
    double coefficient_width = 1280.0 / 640.0;
    double coefficient_height = 1024.0 / 640.0;
    

    // Build the points in the object coordinate system, start from bottom left in
    // clockwise order
    template <typename PointType>
    static inline std::vector<PointType> buildObjectPoints(const double &w,
                                                            const double &h) noexcept {
        return {PointType(0, w / 2, -h / 2),
                PointType(0, w / 2, h / 2),
                PointType(0, -w / 2, h / 2),
                PointType(0, -w / 2, -h / 2)};
     
    }

    //以point2f的形式储存四个关键点，左下开始顺时针，与中南对应
    std::vector<cv::Point2f> points;

};

class OpenVINODetector {

public:
    using DetectorCallback = std::function<void (const std::vector<ArmorObject> &, int64_t,
      const cv::Mat &)>;
private:
    bool infer_callback(Mat img,int detect_color,int64_t timestamp_nanosec);
    float calculate_quad_area(const std::vector<cv::Point2f>& points);
public:
    vector<ArmorObject> objects;
    const int IMAGE_HEIGHT = 640;
    const int IMAGE_WIDTH = 640;
    double ans;
    vector<double> ious;
    vector<ArmorObject> tmp_objects;
    std::shared_ptr<ov::Model> model;
    ov::Core core;
    ov::preprocess::PrePostProcessor *ppp;
    ov::CompiledModel compiled_model;
    ov::Shape input_shape;
    DetectorCallback infer_callback_;

    OpenVINODetector(){}
    OpenVINODetector(string model_path_xml, string model_path_bin, string device);

    

    OpenVINODetector(string model_path, string device){
        input_shape = {1, 1, static_cast<unsigned long>(IMAGE_HEIGHT), static_cast<unsigned long>(IMAGE_WIDTH)};
        // input_shape = { 1, static_cast<unsigned long>(IMAGE_HEIGHT), static_cast<unsigned long>(IMAGE_WIDTH),1};
        model = core.read_model(model_path);
        // Step . Inizialize Preprocessing for the model
        ppp = new ov::preprocess::PrePostProcessor(model);
        // Specify input image format
        ppp->input().tensor().set_shape(input_shape).set_element_type(ov::element::u8).set_layout("NHWC");
        ppp->input().preprocess().resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR);

        //  Specify model's input layout
        ppp->input().model().set_layout("NHWC");
        // Specify output results format
        ppp->output().tensor().set_element_type(ov::element::f32);
        // Embed above steps in the graph
        model = ppp->build();

        compiled_model = core.compile_model(model, device);
    }
    static cv::Mat letterbox(const cv::Mat & img);
    std::future<bool> push_input(const cv::Mat & resize_img, int detect_color_,int64_t timestamp_nanosec);
    void set_callback(DetectorCallback callback);

    void infer(Mat img, float* detections){
        // Step 3. Read input image
        // resize image
        resize(img,img,Size(IMAGE_WIDTH,IMAGE_HEIGHT));
        if(img.channels() == 3) cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);


        // Step 5. Create tensor from image
        // int rows = img.rows;
        // int cols = img.cols;
        uchar* input_data = (uchar *)img.data; // 创建一个新的float数组

        ov::Tensor input_tensor = ov::Tensor(compiled_model.input().get_element_type(), compiled_model.input().get_shape(), input_data);


        // Step 6. Create an infer request for model inference
        ov::InferRequest infer_request = compiled_model.create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();


        //Step 7. Retrieve inference results
        const ov::Tensor &output_tensor = infer_request.get_output_tensor();
        ov::Shape output_shape = output_tensor.get_shape();
        copy(output_tensor.data<float>(), output_tensor.data<float>() + 9, detections);
    }
    double sigmoid(double x) {
        if(x>0)
            return 1.0 / (1.0 + exp(-x));
        else
            return exp(x) / (1.0 + exp(x));
    }

    double cal_iou(const cv::Rect& r1, const cv::Rect& r2)
    {
        float x_left = std::fmax(r1.x, r2.x);
        float y_top = std::fmax(r1.y, r2.y); 
        float x_right = std::fmin(r1.x + r1.width, r2.x + r2.width);
        float y_bottom = std::fmin(r1.y + r1.height, r2.y + r2.height);

        if (x_right < x_left || y_bottom < y_top) {
            return 0.0; 
        }

        double in_area = (x_right - x_left) * (y_bottom - y_top);
        double un_area = r1.area() + r2.area() - in_area; 

        return in_area / un_area;
    }

    double meaning(float x, int len){
        if(len == 0) ans = x;
        else{
            ans = (len * ans + x) / (len+1);
        }
        return ans;
    }

    ~OpenVINODetector(){
        delete ppp;
    }
};

}//namespace rm_auto_aim

#endif //OPENVINO_TEST_OPENVINOINFER_H