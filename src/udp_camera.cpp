#include <cstring>
#include <rclcpp/logger.hpp>
#include <string>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <image_transport/image_transport.hpp>
#include <vector>

#include "buffer_manager.hpp"
#include "safe_queue.hpp"
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

    BufferManager _buffer;
    SafeQueue<std::vector<buff_ptr>> _decode_queue;

    CameraInfo _ci;
    image_transport::CameraPublisher _img_pub;
    TimerBase::SharedPtr _request_timer;

    bool _has_stream;
    bool _select_front_camera;

    std::thread _thr_receive, _thr_decode;

  public:
    UdpCameraNode()
        : Node("udp_camera"),
          _udp(20000),
          _has_stream(false),
          _select_front_camera(true)
    {
        declare_parameter("front", true);
        get_parameter("front", _select_front_camera);
        _img_pub = image_transport::create_camera_publisher(this, "/camera/image_raw");

        _request_timer = create_wall_timer(1s, bind(&UdpCameraNode::check_stream, this));
        request_stream();

        _thr_receive = std::thread(&UdpCameraNode::receive, this);
        _thr_decode = std::thread(&UdpCameraNode::decode, this);
    }

    ~UdpCameraNode()
    {
        if (_thr_receive.joinable())
            _thr_receive.join();
        if (_thr_decode.joinable())
            _thr_decode.join();
    }

    void check_stream()
    {
        if (!_has_stream)
        {
            request_stream();
        }
        _has_stream = false;
    }

    void request_stream()
    {
        if (_select_front_camera)
        {
            RCLCPP_INFO(get_logger(), "Request front");
            _udp.send(_request_front, 2);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Request bottom");
            _udp.send(_request_bottom, 2);
        }
    }

    void receive()
    {
        pthread_setname_np(pthread_self(), "udp receive");

        int current_frame = -1;
        while (ok())
        {
            auto buff = _buffer.get_buffer();
            if (!buff)
            {
                RCLCPP_WARN(get_logger(), "No buffer %d", _decode_queue.size());
                std::this_thread::sleep_for(10ms);
                continue;
            }

            int r;
            while (ok())
            {
                r = _udp.receive(buff->data.begin(), READ_BUFF_SIZE);
                if (r <= 0)
                {
                    std::this_thread::sleep_for(10ms);
                    continue;
                }
                break;
            }
            buff->size = r;

            _buffer.complete(buff);

            int frame_ind = buff->data[0] & 0xff;
            if (current_frame == -1)
            {
                current_frame = frame_ind;
            }
            else if (frame_ind != current_frame)
            {
                RCLCPP_INFO(get_logger(), "Reset %d != %d", frame_ind, current_frame);
                current_frame = frame_ind;
                _buffer.reset();
                continue;
                ;
            }

            if (buff->data[1] == 1) // stream end
            {
                _decode_queue.enqueue(_buffer.pop_completed());
                current_frame = -1;
            }
        }

        RCLCPP_INFO(get_logger(), "Stopping");
        _decode_queue.stop();
    }

    void decode()
    {
        pthread_setname_np(pthread_self(), "jpeg decode");

        while (ok())
        {
            decode_frame();
        }
    }

    void decode_frame()
    {
        auto parts = _decode_queue.dequeue();
        if (!ok())
            return;

        char *buff_offset = _frame_buff;
        int size = 0;

        for (const auto &chunk : parts)
        {
            int cnt = chunk->size - 8;
            std::memcpy(buff_offset, chunk->data.begin() + 8, cnt);
            buff_offset += cnt;
            size += cnt;
        }

        push_frame(size);
        _buffer.return_back(parts);
    }

    void push_frame(int size)
    {
        // RCLCPP_INFO(get_logger(), "Decode %d", size);
        cv::Mat raw(1, size, CV_8UC1, _frame_buff);
        cv::Mat frame = cv::imdecode(raw, cv::IMREAD_COLOR);

        if (!frame.empty())
        {
            // RCLCPP_INFO(get_logger(), "Push frame %dx%d", frame.cols, frame.rows);

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
    }
};

int main(int argc, char *argv[])
{
    init(argc, argv);
    spin(std::make_shared<UdpCameraNode>());
    shutdown();
    return 0;
}
