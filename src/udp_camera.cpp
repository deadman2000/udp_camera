#include <cstring>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <image_transport/image_transport.hpp>

#include "udp_server.hpp"

using namespace rclcpp;
using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace sensor_msgs::msg;

class UdpCameraNode : public Node
{
    UdpServer _udp;
    char _request_front[2] = {0x42, 0x76};
    char _request_bottom[2] = {0x42, 0x79};

    static constexpr int BUFF_SIZE = 1024 * 100;
    char _frame_buff[BUFF_SIZE];
    char *_buff_offset;
    int _read_bytes;

    static constexpr int READ_BUFF_SIZE = 2048;
    char _read_buff[READ_BUFF_SIZE];

    CameraInfo _ci;
    image_transport::CameraPublisher _img_pub;
    TimerBase::SharedPtr _request_timer;

    bool _has_stream;
    bool _select_front_camera;

  public:
    UdpCameraNode()
        : Node("udp_camera"), _udp(20000), _has_stream(false), _select_front_camera(true)
    {
        _img_pub = image_transport::create_camera_publisher(this, "/camera/image_raw");

        _request_timer = create_wall_timer(1s, bind(&UdpCameraNode::request_stream, this));

        std::thread thr(&UdpCameraNode::receive, this);
        thr.detach();
    }

    void request_stream()
    {
        if (!_has_stream)
        {
            RCLCPP_INFO(get_logger(), "Req");
            if (_select_front_camera)
                _udp.send(_request_front, 2);
            else
                _udp.send(_request_bottom, 2);
        }
        _has_stream = false;
    }

    void receive()
    {
        _buff_offset = _frame_buff;
        _read_bytes = 0;
        while (true)
        {
            int r = _udp.receive(_read_buff, READ_BUFF_SIZE);
            if (r <= 0)
            {
                std::this_thread::sleep_for(10ms);
                continue;
            }
            int cnt = r - 8;

            if (_read_bytes + cnt > BUFF_SIZE)
            {
                RCLCPP_WARN(get_logger(), "Buffer overflow %d > %d", _read_bytes + cnt, BUFF_SIZE);
                buffer_reset();
                continue;
            }

            std::memcpy(_buff_offset, _read_buff + 8, cnt);
            _buff_offset += cnt;
            _read_bytes += cnt;

            if (_read_buff[1] == 1)
            {
                RCLCPP_INFO(get_logger(), "Decode %d", _read_bytes);
                cv::Mat raw(1, _read_bytes, CV_8UC1, _frame_buff);
                cv::Mat frame = cv::imdecode(raw, cv::IMREAD_COLOR);

                if (!frame.empty())
                {
                    RCLCPP_INFO(get_logger(), "Push frame %dx%d", frame.cols, frame.rows);

                    Image img;
                    img.encoding = "bgr8";
                    img.is_bigendian = false;
                    img.header.stamp = now();
                    img.width = frame.cols;
                    img.height = frame.rows;
                    img.step = static_cast<Image::_step_type>(frame.step);
                    img.data.resize(img.step * img.height);
                    std::memcpy(img.data.data(), frame.datastart, img.data.size());

                    _ci.width = frame.cols;
                    _ci.height = frame.rows;
                    _img_pub.publish(img, _ci);
                }

                _has_stream = true;
                buffer_reset();
            }
        }
    }

    void buffer_reset()
    {
        _buff_offset = _frame_buff;
        _read_bytes = 0;
    }
};

int main(int argc, char *argv[])
{
    init(argc, argv);
    spin(std::make_shared<UdpCameraNode>());
    shutdown();
    return 0;
}
