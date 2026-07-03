#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include "mors_msgs/Body.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/height_map_msg.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <chrono>
#include <vector>
#include <algorithm>

#define dSIZE_X 1.5
#define dSIZE_Y 1.0
#define dRESOLUTION 0.005

mors_msgs::Body current_odom;
// typedef Eigen::Matrix<float, 3, 3> Matrix3f;
// typedef Eigen::Matrix<float, 3, 1> Vector3f;

template<typename T>
std::vector<std::vector<T>> shiftMatrix(const std::vector<std::vector<T>>& src, int dx, int dy, T fillValue = T{}) {
    int X_CELL = static_cast<int>(src.size());
    int Y_CELL = static_cast<int>(src[0].size());

    // Создаём результирующую матрицу того же размера, заполненную fillValue
    std::vector<std::vector<T>> dst(X_CELL, std::vector<T>(Y_CELL, fillValue));

    for (int x = 0; x < X_CELL; x++) {
        for (int y = 0; y < Y_CELL; y++) {
            // Вычисляем, откуда брать значение в исходной матрице
            int srcX = x - dx;  // сдвиг содержимого: если dx=+50, то (50,0) ← (0,0)
            int srcY = y - dy;

            // Проверяем, не вышли ли за границы исходной матрицы
            if (srcX >= 0 && srcX < X_CELL && srcY >= 0 && srcY < Y_CELL) {
                dst[x][y] = src[srcX][srcY];
            }
            // Иначе остаётся fillValue
        }
    }

    return dst;
}
// float** shiftMatrix(const float* src, int X_CELL, int Y_CELL, int dx, int dy, float fillValue) {
//     // Создаём результирующую матрицу того же размера, заполненную fillValue
//     float dst[X_CELL*Y_CELL] = {fillValue};

//     for (int x = 0; x < X_CELL; x++) {
//         for (int y = 0; y < Y_CELL; y++) {
//             // Вычисляем, откуда брать значение в исходной матрице
//             int srcX = x - dx;  // потому что мы двигаем содержимое: если dx=+50, то в (50,0) должно быть то, что было в (0,0)
//             int srcY = y - dy;

//             // Проверяем, не вышли ли за границы исходной матрицы
//             if (srcX >= 0 && srcX < X_CELL && srcY >= 0 && srcY < Y_CELL) {
//                 dst[x*X_CELL+y] = src[x*X_CELL+y];
//             }
//             // Иначе остаётся fillValue
//         }
//     }

//     return static_cast<>(dst);
// }

class Handler
{
    public:
        void handleMessage(const lcm::ReceiveBuffer *rbuf,
            const std::string &chan, 
            const mors_msgs::robot_state_msg *msg)
    {
        
        current_odom = msg->body;
        // std::cout << "Received message on channel " << chan << std::endl;

        // std::cout << "Position: " << msg->position[0] << " " << msg->position[1] << " " <<msg->position[2] << std::endl;
        // std::cout << "orientation: " << msg->orientation[0] << " " << msg->orientation[1] << " " <<msg->orientation[2] << std::endl;
        // std::cout << "lin_vel: " << msg->lin_vel[0] << " " << msg->lin_vel[1] << " " <<msg->lin_vel[2] << std::endl;
        // std::cout << "ang_vel: " << msg->ang_vel[0] << " " << msg->ang_vel[1] << " " <<msg->ang_vel[2] << std::endl;

    }
};

float r11, r12, r13, r21, r22, r23, r31, r32, r33;
Eigen::Matrix3f calc_rotating_matrix(float roll, float pitch, float yaw) // rotating about xyz axes
{
    r11 = std::cos(yaw)*std::cos(pitch);
    r12 = std::cos(yaw)*std::sin(pitch)*std::sin(roll) - std::sin(yaw)*std::cos(roll);
    r13 = std::cos(yaw)*std::sin(pitch)*std::cos(roll) + std::sin(yaw)*std::sin(roll);
    r21 = std::sin(yaw)*std::cos(pitch);
    r22 = std::sin(yaw)*std::sin(pitch)*std::sin(roll) + std::cos(yaw)*std::cos(roll);
    r23 = std::sin(yaw)*std::sin(pitch)*std::cos(roll) - std::cos(yaw)*std::sin(roll);
    r31 = -std::sin(pitch);
    r32 = std::cos(pitch)*std::sin(roll);
    r33 = std::cos(pitch)*std::cos(roll);

    Eigen::Matrix3f R;
    R << r11, r12, r13,
         r21, r22, r23,
         r31, r32, r33;
    return R;
}


int main(int argc, char **argv)
{
    std::cout<<"vers:0.1"<<std::endl;
    rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);
    pipe.start(cfg);

    rs2::decimation_filter dec_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);

    lcm::LCM lcm;
    if (!lcm.good()){return 1;}

    std::cout << "OK!" << std::endl;

    Handler handlerObject;
    lcm.subscribe("ROBOT_STATE", &Handler::handleMessage, &handlerObject);

    if (lcm.handle() != 0)
    {exit(-1);}

    mors_msgs::Body start_odom = current_odom;
    

    // while(0 == lcm.handle());

    // rs2::frameset frames = pipe.wait_for_frames();
    rs2::frameset frames;
    Eigen::Matrix3f base_rotate = calc_rotating_matrix(-(M_PI_2+36.6*M_PI/180), 0, M_PI_2);
    Eigen::Vector3f base_vector;
    base_vector << (244+4+7.06+16.74)/1000, 17.5/1000, 12.43/1000;
    std::cout<<"GO"<<std::endl;

    // mors_msgs::height_map_msg height_map;
    // std::vector<std::vector<float>> height_map;
    std::vector<std::vector<float>> height_map(dSIZE_X/dRESOLUTION, std::vector<float>(dSIZE_Y/dRESOLUTION, 0.0));
    // for (int i=0; i<int(dSIZE_X/dRESOLUTION);i++)
    // {
    //     for (int j=0; j<int(dSIZE_Y/dRESOLUTION);j++)
    //     {
    //         // height_map.height_map[i][j] = 0;
    //         height_map[i][j] = 0;
    //     }
    // }
    // int move_hm_dx, move_hm_dy;
    Eigen::Vector3f prev_pose_vector;
    prev_pose_vector << 0,0,0;

    while (lcm.handle()==0)
    {
        // std::cout<<"wait"<<std::endl;
        // frames = pipe.wait_for_frames();
        if (pipe.poll_for_frames(&frames)==0){continue;}

        if (rs2::frame depth = frames.get_depth_frame())
        {


            // std::cout<<"getted"<<std::endl;
            float roll = current_odom.orientation[0]-start_odom.orientation[0];
            float pitch = current_odom.orientation[1]-start_odom.orientation[1];
            float yaw = current_odom.orientation[2]-start_odom.orientation[2];
            // std::cout<<"Yaw: "<<yaw<<std::endl;
            // Eigen::Matrix3f R = calc_rotating_matrix(0, 0, yaw);
            // Eigen::Matrix3f R = calc_rotating_matrix(0, -pitch, 0);
            // Eigen::Matrix3f R = calc_rotating_matrix(-roll, 0, 0);
            Eigen::Matrix3f R = calc_rotating_matrix(-roll, -pitch, yaw);

            points = pc.calculate(depth);
            const rs2::vertex *vertices = points.get_vertices();
            Eigen::Vector3f vertices_vector, pose_vector;
            pose_vector << current_odom.position[0]-start_odom.position[0],
                            current_odom.position[1]-start_odom.position[1],
                            -(current_odom.position[2]-start_odom.position[2]);
            // std::cout<<"zeroed"<<std::endl;
            Eigen::Vector3f result;
            for (size_t i=0; i<points.size(); i++)
            {
                vertices_vector << vertices[i].x, vertices[i].y, vertices[i].z;

                // to robot coords
                result = base_rotate*vertices_vector;
                result = result - base_vector;

                // to world coords
                result = R*result;
                result = result - pose_vector;

                
                int ind_x = int((result[0] + dSIZE_X/2.0)/dRESOLUTION);
                int ind_y = int((result[1] + dSIZE_Y/2.0)/dRESOLUTION);
                // int ind_x = int(result[0]/height_map.RESOLUTION);
                // int ind_y = int(result[1]/height_map.RESOLUTION);

                if ((ind_x<0) || (ind_x>=int(dSIZE_X/dRESOLUTION)) || (ind_y<0) || (ind_y>=int(dSIZE_Y/dRESOLUTION)) )
                {continue;}

                height_map[ind_x][ind_y] = result[2];
            }

            // for (int i=0; i<100; i++){height_map.height_map[i][0]=i/1000.0;}
            // for (int i=0; i<100; i+=2){height_map.height_map[0][i]=i/1000.0;}
            // std::cout<<(pose_vector[0]-prev_pose_vector[0])<<" "<<(pose_vector[0]-prev_pose_vector[0])/dRESOLUTION<<std::endl;
            if (int((pose_vector[0]-prev_pose_vector[0])/dRESOLUTION)!=0)
            {
                height_map = shiftMatrix<float>(height_map, int((pose_vector[0]-prev_pose_vector[0])/dRESOLUTION), 0, 0.0);
                prev_pose_vector[0] = pose_vector[0];
                // std::cout<<"x"<<std::endl;
            }
            if (int((pose_vector[1]-prev_pose_vector[1])/dRESOLUTION)!=0)
            {
                height_map = shiftMatrix<float>(height_map, 0, int((pose_vector[1]-prev_pose_vector[1])/dRESOLUTION), 0.0);
                prev_pose_vector[1] = pose_vector[1];
                // std::cout<<"y"<<std::endl;
            }

            mors_msgs::height_map_msg height_map_msg;
            for (int i=0;i<(dSIZE_X/dRESOLUTION); i++)
                {
                    for (int j=0;j<(dSIZE_Y/dRESOLUTION);j++)
                    {
                        height_map_msg.height_map[i][j] = height_map[i][j];
                    }
                }
            auto current_time = std::chrono::system_clock::now();
            auto duration = current_time.time_since_epoch();
            auto epoch_time = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
            height_map_msg.timestamp = epoch_time;
            // std::cout<<"publi"<<std::endl;
            lcm.publish("HEIGHT_MAP", &height_map_msg);
            // std::cout<<"Send!" << std::endl;
        }
    }
    return 0;
}