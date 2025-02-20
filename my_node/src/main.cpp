#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/proto/map_builder_options.pb.h>
#include <cartographer/sensor/point_cloud.h>
#include <cartographer/sensor/timed_point_cloud_data.h>
#include <cartographer/io/proto_stream.h>
#include <cartographer/io/proto_stream_deserializer.h>

#include <iostream>
#include <vector>

using namespace cartographer;

// 生成一个简单的激光雷达数据
sensor::TimedPointCloudData GenerateFakeLaserScan() {
    sensor::TimedPointCloud point_cloud;
    for (int i = 0; i < 360; i += 10) {
        float angle = i * M_PI / 180.0;  // 角度转弧度
        float range = 5.0 + (rand() % 100) / 100.0; // 5m 基础范围，增加 0~1m 随机值
        //point_cloud.push_back({range * cos(angle), range * sin(angle), 0.0f, 0.0f});
        point_cloud.emplace_back({range * cos(angle), range * sin(angle), 0.0f, 0.0f});
    }

    return sensor::TimedPointCloudData{common::FromUniversal(0), Eigen::Vector3f::Zero(), point_cloud};
}

int main() {
    // 1. 创建 MapBuilder
    //mapping::proto::MapBuilderOptions map_builder_options;
    //map_builder_options.set_use_trajectory_builder_2d(true);

    //auto map_builder_options = mapping::CreateMapBuilderOptions(lua_parameter_dictionary.GetDictionary("map_builder").get());
    //auto trajectory_builder_options = mapping::CreateTrajectoryBuilderOptions(lua_parameter_dictionary.GetDictionary("trajectory_builder").get());
    auto map_builder_options = mapping::CreateMapBuilderOptions(true);
    auto trajectory_builder_options = mapping::CreateTrajectoryBuilderOptions(true);

    mapping::MapBuilder map_builder(map_builder_options);

    using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
    using SensorType = SensorId::SensorType;
    std::set<SensorId> sensor_ids;
    sensor_ids.insert(SensorId{SensorType::RANGE, "range0"});

    //auto* trajectory = map_builder.AddTrajectoryBuilder({}, nullptr);
    auto* trajectory = map_builder.AddTrajectoryBuilder(sensor_ids, trajectory_builder_options);
    /*
    trajectory_id = map_builder->AddTrajectoryBuilder(sensor_ids, trajectory_builder_options,
                                                      [this](const int id,
                                                             const ::cartographer::common::Time time,
                                                             const transform::Rigid3d local_pose,
                                                             sensor::RangeData range_data_in_local,
                                                             const std::unique_ptr<const ::cartographer::mapping::TrajectoryBuilderInterface::InsertionResult> res) {
                                                          OnLocalSlamResult2(id, time, local_pose, range_data_in_local);
                                                      });
    */

    // 2. 生成 100 组激光数据并处理
    for (int i = 0; i < 100; ++i) {
        sensor::TimedPointCloudData laser_scan = GenerateFakeLaserScan();
        trajectory->AddSensorData("laser", laser_scan);
    }

    // 3. 结束并导出地图
    std::cout << "SLAM 计算完成，导出地图..." << std::endl;
    mapping::proto::SerializedData serialized_data;
    //map_builder.SerializeStateToProto(&serialized_data);
    map_builder.SerializeStateToFile(true, "map.pbstream");
    
    return 0;
}

