// #include <rclcpp/rclcpp.hpp>
// #include <grid_map_ros/grid_map_ros.hpp>
// #include <grid_map_msgs/msg/grid_map.hpp>
#include <cmath>
#include <memory>
#include <utility>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <string>

struct float3 {
  float x, y, z;
  float3 operator*(float t)
  {
      return { x * t, y * t, z * t };
  }

  float3 operator-(float t)
  {
      return { x - t, y - t, z - t };
  }

  void operator*=(float t)
  {
      x = x * t;
      y = y * t;
      z = z * t;
  }

  // void operator=(float3 other)
  // {
  //     x = other.x;
  //     y = other.y;
  //     z = other.z;
  // }

  void add(float t1, float t2, float t3)
  {
      x += t1;
      y += t2;
      z += t3;
  }
};

class rotation_estimator
{
    // theta is the angle of camera rotation in x, y and z components
    float3 theta;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha = 0.98f;
    bool firstGyro = true;
    bool firstAccel = true;
    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;
public:
    // Function to calculate the change in angle of motion based on data from gyro
    void process_gyro(rs2_vector gyro_data, double ts)
    {
        if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
        {
            firstGyro = false;
            last_ts_gyro = ts;
            return;
        }
        // Holds the change in angle, as calculated from gyro
        float3 gyro_angle;

        // Initialize gyro_angle with data from gyro
        gyro_angle.x = gyro_data.x; // Pitch
        gyro_angle.y = gyro_data.y; // Yaw
        gyro_angle.z = gyro_data.z; // Roll

        // Compute the difference between arrival times of previous and current gyro frames
        double dt_gyro = (ts - last_ts_gyro) / 1000.0;
        last_ts_gyro = ts;

        // Change in angle equals gyro measures * time passed since last measurement
        gyro_angle = gyro_angle * static_cast<float>(dt_gyro);

        theta.add(gyro_angle.x, gyro_angle.y, gyro_angle.z);
    }

    void process_accel(rs2_vector accel_data)
    {
        // Holds the angle as calculated from accelerometer data
        float3 accel_angle;

        // Calculate rotation angle from accelerometer data
        accel_angle.x = atan2(accel_data.y, accel_data.z);
        accel_angle.z = atan2(-accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        if (firstAccel)
        {
            firstAccel = false;
            theta = accel_angle;
            // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
            theta.y = 0;
        }
        else
        {
            /* 
            Apply Complementary Filter:
                - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                  that are steady over time, is used to cancel out drift.
                - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
            */
            theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
            theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
        }
    }
    
    // Returns the current rotation angle
    float3 get_theta()
    { return theta; }
};





int main(int argc, char ** argv)
{
  std::cout<<"vers:0.6"<<std::endl;
  rs2::pointcloud pc;
  rs2::points points;
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
  cfg.enable_stream(RS2_STREAM_DEPTH);
  pipe.start(cfg);

  rs2::decimation_filter dec_filter;
  dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);

  double size_x = 1.5;
  double size_y = 1.5;
  double resolution = 0.005;


  // Initialize node and publisher.
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("grid_map_simple_demo");
  auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", rclcpp::QoS(1).transient_local());

  // Create grid map.
  grid_map::GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(size_x, size_y), resolution);
  RCLCPP_INFO(
    node->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  // Work with grid map in a loop.
  rclcpp::Rate rate(10.0);
  rclcpp::Clock clock;
  rotation_estimator algo;

  rs2::frameset frames = pipe.wait_for_frames();

  if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
  {
      double ts = gyro_frame.get_timestamp();
      rs2_vector gyro_data = gyro_frame.get_motion_data();
      algo.process_gyro(gyro_data, ts);
  }

  if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
  {
    rs2_vector accel_data = accel_frame.get_motion_data();
    algo.process_accel(accel_data);
  }

  frames = pipe.wait_for_frames();

  if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
  {
      double ts = gyro_frame.get_timestamp();
      rs2_vector gyro_data = gyro_frame.get_motion_data();
      algo.process_gyro(gyro_data, ts);
  }

  if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
  {
    rs2_vector accel_data = accel_frame.get_motion_data();
    algo.process_accel(accel_data);
  }

  float3 theta = algo.get_theta();
  float3 theta_start = theta;


    // double R[3][3] = {
    // {std::cos(theta.z)*std::cos(theta.y), std::cos(theta.z)*std::sin(theta.y)*std::sin(theta.x)-std::sin(theta.z)*std::cos(theta.x), std::cos(theta.z)*std::sin(theta.y)*std::cos(theta.x)+std::sin(theta.z)*std::sin(theta.x)},
    // {std::sin(theta.z)*std::cos(theta.y), std::sin(theta.z)*std::sin(theta.y)*std::sin(theta.x)+std::cos(theta.z)*std::cos(theta.x), std::sin(theta.z)*std::sin(theta.y)*std::cos(theta.x)-std::cos(theta.z)*std::sin(theta.x)},
    // {-std::sin(theta.y), std::cos(theta.y)*std::sin(theta.x), std::cos(theta.y)*std::cos(theta.x)}
    // };

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
  { map.at("elevation", *it) = 0; }
  while (rclcpp::ok()) {
    frames = pipe.wait_for_frames();

    if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
    {
        double ts = gyro_frame.get_timestamp();
        rs2_vector gyro_data = gyro_frame.get_motion_data();
        algo.process_gyro(gyro_data, ts);
    }

    if (rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL))
    {
      rs2_vector accel_data = accel_frame.get_motion_data();
      algo.process_accel(accel_data);
    }
 

    // std::cout<<"Theta: "<<theta.x<<" "<<theta.y<<" "<<theta.z<<std::endl;  
    // std::cout<<"1"<<std::endl;
    if (rs2::frame depth = frames.get_depth_frame())
    {
      // depth = dec_filter.process(depth);
      float3 theta = algo.get_theta();
      float alpha = theta.z - theta_start.z;
    
      float beta =  theta.y - theta_start.y;
      float gamma = theta.x - theta_start.x;
      // std::cout<<"alpha(z): "<<alpha<<", beta(y): "<<beta<<", gamma(x): "<<gamma<<std::endl;
      
      float R_z[3][3] =
      {
        {std::cos(alpha), -std::sin(alpha), 0},
        {std::sin(alpha), std::cos(alpha), 0},
        {0, 0, 1}
      };
      float R_x[3][3] =
      {
        {1, 0, 0},
        {0, std::cos(gamma), -std::sin(gamma)},
        {0, std::sin(gamma), std::cos(gamma)}
      };
  
      float R_y[3][3] =
      {
        {std::cos(beta), 0, std::sin(beta)},
        {0, 1, 0},
        {-std::sin(beta), 0, std::cos(beta)}
      };
  
      // double R[3][3] = {
      //   {std::cos(theta.z)*std::cos(theta.y), std::cos(theta.z)*std::sin(theta.y)*std::sin(theta.x)-std::sin(theta.z)*std::cos(theta.x), std::cos(theta.z)*std::sin(theta.y)*std::cos(theta.x)+std::sin(theta.z)*std::sin(theta.x)},
      //   {std::sin(theta.z)*std::cos(theta.y), std::sin(theta.z)*std::sin(theta.y)*std::sin(theta.x)+std::cos(theta.z)*std::cos(theta.x), std::sin(theta.z)*std::sin(theta.y)*std::cos(theta.x)-std::cos(theta.z)*std::sin(theta.x)},
      //   {-std::sin(theta.y), std::cos(theta.y)*std::sin(theta.x), std::cos(theta.y)*std::cos(theta.x)}
      //   };

      points = pc.calculate(depth);
      const rs2::vertex *vertices = points.get_vertices();


      
      // std::cout<<"2"<<std::endl;
      for (size_t i=0; i < points.size(); i++)
      {
        float vert_ar[3] = {vertices[i].x, vertices[i].y, vertices[i].z};
        float calc_vert_ar[3] = {0};
        // float calc_vert_ar[3] = {vertices[i].x, vertices[i].y, vertices[i].z};

        // std::cout<<"3"<<std::endl;

        for (int i=0; i<3; i++) {for (int j=0; j<3;j++) {calc_vert_ar[i] += R_z[i][j] * vert_ar[j];} }
        vert_ar[0]=calc_vert_ar[0]; vert_ar[1]=calc_vert_ar[1]; vert_ar[2]=calc_vert_ar[2]; calc_vert_ar[0]=0; calc_vert_ar[1]=0; calc_vert_ar[2]=0;

        for (int i=0; i<3; i++) {for (int j=0; j<3;j++) {calc_vert_ar[i] += R_y[i][j] * vert_ar[j];} }
        vert_ar[0]=calc_vert_ar[0]; vert_ar[1]=calc_vert_ar[1]; vert_ar[2]=calc_vert_ar[2]; calc_vert_ar[0]=0; calc_vert_ar[1]=0; calc_vert_ar[2]=0;

        for (int i=0; i<3; i++) {for (int j=0; j<3;j++) {calc_vert_ar[i] += R_x[i][j] * vert_ar[j];} }
        vert_ar[0]=calc_vert_ar[0]; vert_ar[1]=calc_vert_ar[1]; vert_ar[2]=calc_vert_ar[2]; calc_vert_ar[0]=0; calc_vert_ar[1]=0; calc_vert_ar[2]=0;
        beta = -3.1415926 / 2;
        gamma = -3.1415926 / 2;
        alpha = 0;

        float R_x[3][3] =
        {
          {1, 0, 0},
          {0, std::cos(gamma), -std::sin(gamma)},
          {0, std::sin(gamma), std::cos(gamma)}
        };
    
        float R_y[3][3] =
        {
          {std::cos(beta), 0, std::sin(beta)},
          {0, 1, 0},
          {-std::sin(beta), 0, std::cos(beta)}
        };
      
        float R_z[3][3] =
        {
          {std::cos(alpha), -std::sin(alpha), 0},
          {std::sin(alpha), std::cos(alpha), 0},
          {0, 0, 1}
        };
        for (int i=0; i<3; i++) {for (int j=0; j<3;j++) {calc_vert_ar[i] += R_z[i][j] * vert_ar[j];} }
        vert_ar[0]=calc_vert_ar[0]; vert_ar[1]=calc_vert_ar[1]; vert_ar[2]=calc_vert_ar[2]; calc_vert_ar[0]=0; calc_vert_ar[1]=0; calc_vert_ar[2]=0;

        for (int i=0; i<3; i++) {for (int j=0; j<3;j++) {calc_vert_ar[i] += R_y[i][j] * vert_ar[j];} }
        vert_ar[0]=calc_vert_ar[0]; vert_ar[1]=calc_vert_ar[1]; vert_ar[2]=calc_vert_ar[2]; calc_vert_ar[0]=0; calc_vert_ar[1]=0; calc_vert_ar[2]=0;

        for (int i=0; i<3; i++) {for (int j=0; j<3;j++) {calc_vert_ar[i] += R_x[i][j] * vert_ar[j];} }
        vert_ar[0]=calc_vert_ar[0]; vert_ar[1]=calc_vert_ar[1]; vert_ar[2]=calc_vert_ar[2]; calc_vert_ar[0]=0; calc_vert_ar[1]=0; calc_vert_ar[2]=0;


        int ind_x = int((vert_ar[0] + size_x/2)/resolution);
        int ind_y = int((vert_ar[1] + size_y/2)/resolution);

        // std::cout<<"5"<<std::endl;
        if ((ind_x<0) || (ind_x>=int(size_x/resolution)) || (ind_y<0) || (ind_y>=int(size_y/resolution)) )
          {continue;}
        
        // std::cout<<"6"<<std::endl;
        grid_map::Index ind (ind_x, ind_y); // Инвертирование лево/право
        // std::cout<<i<<" 7 "<< x << " "<< y<< " "<<z<< std::endl;
        vert_ar[2] = vert_ar[2]+0.215;
        
        map.at("elevation", ind) = vert_ar[2];
        // map.at("elevation", ind) = calc_vert_ar[2];

        // std::cout<<"8"<<std::endl;
      }
      std::string pr = "";
      int from_x = (size_x/resolution)/2-110;
      int from_y = (size_y/resolution)/2-30;
      int to_y = (size_y/resolution)/2+30;
      int to_x = (size_x/resolution)/2-70;
      // for (int i=from_x; i<to_x;i++)
      // {
      //   for (int j=from_y; j<to_y; j++)
      //     {
      //       grid_map::Index ind (i,j);
      //       char tmp[5] = {0};
      //       std::sprintf(tmp, "%3d", int(map.at("elevation", ind)*1000));
      //       pr += std::string(tmp);

      //       if ((i==from_x) || (i==to_x-1) || (j==from_y) || (j==to_y-1))
      //         map.at("elevation", ind) = 0.1;

      //     }
      //     pr += '\n';
      // }
      // std::cout<<pr<<std::endl<<std::endl;
      // Publish grid map.
      rclcpp::Time time = node->now();
      map.setTimestamp(time.nanoseconds());
      std::unique_ptr<grid_map_msgs::msg::GridMap> message;
      message = grid_map::GridMapRosConverter::toMessage(map);
      publisher->publish(std::move(message));
      // RCLCPP_INFO_THROTTLE(node->get_logger(), clock, 1000, "Grid map published.");

      // Wait for next cycle.
      
  }
  rate.sleep();
  }

  return 0;
}