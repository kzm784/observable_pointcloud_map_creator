#include "observable_pointcloud_map_creator/observable_pointcloud_map_creator.hpp"

ObservablePointcloudMapCreator::ObservablePointcloudMapCreator(const rclcpp::NodeOptions & options)
: Node("observable_pointcloud_map_creator", options)
{
    // Declare parameters
    declare_parameter<std::string>("input_pcd_path", "");
    declare_parameter<std::string>("output_dir", "output");
    declare_parameter<double>("grid_size", 30.0);
    declare_parameter<double>("voxel_leaf_size", 1.0);
    declare_parameter<double>("hpr_radius", 50.0);

    // Get parameters
    get_parameter("input_pcd_path", input_pcd_path_);
    get_parameter("output_dir", output_dir_);
    get_parameter("grid_size", grid_size_);
    get_parameter("voxel_leaf_size", voxel_leaf_size_);
    get_parameter("hpr_radius", hpr_radius_);

    RCLCPP_INFO(get_logger(), "[Init] input_pcd_path: %s", input_pcd_path_.c_str());
    RCLCPP_INFO(get_logger(), "[Init] output_dir: %s", output_dir_.c_str());
    RCLCPP_INFO(get_logger(), "[Init] grid_size=%.2f, voxel_leaf_size=%.2f, hpr_radius=%.2f", grid_size_, voxel_leaf_size_, hpr_radius_);

    std::filesystem::create_directories(output_dir_);

    // Debug
    // std::filesystem::create_directories(output_dir_ + "/debug");

    // Load input PCD
    original_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile(input_pcd_path_, *original_cloud_) < 0) {
        RCLCPP_FATAL(get_logger(), "[Error] Failed to load PCD: %s", input_pcd_path_.c_str());
        return;
    }
    RCLCPP_INFO(get_logger(), "[Init] Loaded %zu points from PCD", original_cloud_->size());

    // Remove outliers globally
    RCLCPP_INFO(get_logger(), "[SOR] Starting global outlier removal");
    filtered_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(original_cloud_);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*filtered_cloud_);
    RCLCPP_INFO(get_logger(), "[SOR] After removal: %zu points", filtered_cloud_->size());

    // Extract ground points
    RCLCPP_INFO(get_logger(), "[PMF] Extracting ground points");
    std::vector<int> ground_indices;
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud(filtered_cloud_);
    pmf.setMaxWindowSize(20);
    pmf.setSlope(1.0f);
    pmf.setInitialDistance(0.5f);
    pmf.setMaxDistance(3.0f);
    pmf.extract(ground_indices);

    ground_global_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (int idx : ground_indices) {
        if (idx >= 0 && idx < static_cast<int>(filtered_cloud_->size())) {
            ground_global_->push_back(filtered_cloud_->points[idx]);
        }
    }
    RCLCPP_INFO(get_logger(), "[PMF] Ground points count: %zu", ground_global_->size());

    // Debug
    // std::ostringstream oss;    
    // oss << output_dir_ << "/debug/ground_global.pcd";
    // pcl::io::savePCDFileBinary(oss.str(), *ground_global_);
    // RCLCPP_DEBUG(get_logger(), "[PMF] Saved %zu points to %s", ground_global_->size(), oss.str().c_str());

    // Start processing each grid cell
    RCLCPP_INFO(get_logger(), "[Grid] Splitting into cells and processing");
    splitAndProcessGrid(ground_global_);

    RCLCPP_INFO(get_logger(), "[Grid] All process has done.");
}

void ObservablePointcloudMapCreator::splitAndProcessGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
    RCLCPP_INFO(get_logger(), "[Grid] Enter splitAndProcessGrid");
    if (!cloud || cloud->empty()) {
        RCLCPP_ERROR(get_logger(), "[Grid] Empty cloud received");
        return;
    }

    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    int nx = std::max(1, static_cast<int>(std::ceil((maxPt.x - minPt.x) / grid_size_)));
    int ny = std::max(1, static_cast<int>(std::ceil((maxPt.y - minPt.y) / grid_size_)));
    RCLCPP_INFO(get_logger(), "[Grid] nx=%d, ny=%d", nx, ny);

    for (int ix = 0; ix < nx; ++ix) {
        for (int iy = 0; iy < ny; ++iy) {
            RCLCPP_DEBUG(get_logger(), "[Grid] Processing cell (%d,%d)", ix, iy);
            Eigen::Vector4f min_box(minPt.x + ix*grid_size_, minPt.y + iy*grid_size_, minPt.z - 1.0f, 1.0f);
            Eigen::Vector4f max_box(minPt.x + (ix+1)*grid_size_, minPt.y + (iy+1)*grid_size_, maxPt.z + 1.0f, 1.0f);

            pcl::CropBox<pcl::PointXYZ> crop;
            crop.setInputCloud(cloud);
            crop.setMin(min_box);
            crop.setMax(max_box);

            auto local = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            crop.filter(*local);
            RCLCPP_DEBUG(get_logger(), "[Grid] Cell (%d,%d) local size: %zu", ix, iy, local->size());
            if (local->size() < 3) continue;

            // Debug
            // std::ostringstream oss;
            // oss << output_dir_ << "/debug/ground_local_" << ix << "_" << iy << ".pcd";
            // pcl::io::savePCDFileBinary(oss.str(), *local);
            // RCLCPP_DEBUG(get_logger(), "[Grid] Saved %zu points to %s", local->size(), oss.str().c_str());

            auto result = processLocalGrid(local);
            RCLCPP_DEBUG(get_logger(), "[Grid] Cell (%d,%d) result size: %zu", ix, iy, result->size());
            if (result && !result->empty()) {
                std::ostringstream oss;
                
                // Debug
                // ボクセルグリッドフィルタリングされた地面点群
                // oss << output_dir_ << "/debug/ground_local_filtered_" << ix << "_" << iy << ".pcd";
                
                // HPR適用後
                oss << output_dir_ << "/grid_" << ix << "_" << iy << ".pcd";
                pcl::io::savePCDFileBinary(oss.str(), *result);
                RCLCPP_INFO(get_logger(), "[Grid] Saved %zu points to %s", result->size(), oss.str().c_str());
            }
        }
    }
    // Write meta data file
    std::ofstream ofs(output_dir_ + "/grid_meta_data.yaml");
    ofs << "grid_size: " << grid_size_ << "\n";
    ofs << "min_pt:\n";
    ofs << "  x: " << minPt.x << "\n";
    ofs << "  y: " << minPt.y << "\n";
    ofs << "max_pt:\n";
    ofs << "  x: " << maxPt.x << "\n";
    ofs << "  y: " << maxPt.y << "\n";
    ofs.close();

    RCLCPP_INFO(get_logger(), "[Meta] Wrote grid_meta_data.yaml");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObservablePointcloudMapCreator::processLocalGrid(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & local)
{
    RCLCPP_DEBUG(get_logger(), "[HPR] Enter processLocalGrid, local size: %zu", local->size());

    // Downsample local grid
    auto vg_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(local);
    vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    vg.filter(*vg_cloud);
    RCLCPP_DEBUG(get_logger(), "[HPR] After VoxelGrid: %zu points", vg_cloud->size());
    if (vg_cloud->empty()) return vg_cloud;
    
    // Debug
    // ボクセルグリッドフィルタされた地面点群だけ見たいとき
    // return vg_cloud;

    // Prepare Open3D cloud for HPR
    open3d::geometry::PointCloud o3d_global;
    o3d_global.points_.reserve(original_cloud_->size());
    for (const auto &p : original_cloud_->points) {
        o3d_global.points_.emplace_back(p.x, p.y, p.z);
    }

    // Hidden Point Removal
    RCLCPP_DEBUG(get_logger(), "[HPR] Performing Hidden Point Removal with radius %.2f", hpr_radius_);
    std::unordered_set<size_t> visible_idx;
    for (size_t i = 0; i < vg_cloud->size(); ++i) {
        const auto &vp = vg_cloud->points[i];
        try {
            auto [mesh, indices] = o3d_global.HiddenPointRemoval(
            Eigen::Vector3d(vp.x, vp.y, vp.z + 1.0), hpr_radius_);
            // RCLCPP_INFO(get_logger(), "[HPR] Viewpoint %zu gave %zu indices", i, indices.size());
            for (size_t id : indices) {
                if (id < original_cloud_->size()) visible_idx.insert(id);
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "[HPR] Exception at viewpoint %zu: %s", i, e.what());
            continue;
        }
    }
    RCLCPP_DEBUG(get_logger(), "[HPR] Total visible points: %zu", visible_idx.size());

    // Build result
    auto result = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    result->reserve(visible_idx.size());
    for (auto id : visible_idx) {
        result->push_back(original_cloud_->points[id]);
    }
    result->width  = static_cast<uint32_t>(result->size());
    result->height = 1;
    result->is_dense = false;
    RCLCPP_DEBUG(get_logger(), "[HPR] Returning result with %zu points", result->size());
    return result;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ObservablePointcloudMapCreator);