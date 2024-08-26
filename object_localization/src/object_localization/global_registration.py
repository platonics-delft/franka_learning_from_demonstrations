import sys
import rospy
import open3d as o3d
import numpy as np
import logging
from time import perf_counter


class GlobalRegistration():
    _source_file: str
    _target_file: str
    _target_pcl: o3d.geometry.PointCloud
    _transformation: np.ndarray
    _logger: logging.Logger
    _voxel_size: float = 0.001
    _normal_radius: float = 0.002
    _search_radius: float = 0.005
    _distance_threshold: float = 0.01
    _max_nn_normal: int = 30
    _max_nn_fpfh: int = 100
    _using_icp: bool = True


    def __init__(self, source_file: str, target_file: str):
        self._source_file = source_file
        self.target_file = target_file
        self._transformation = np.identity(4)
        self._logger = logging.getLogger(__name__)
        self._logger.setLevel(logging.WARN)

    def set_log_level(self, level: int):
        self._logger.setLevel(level)

    @property
    def voxel_size(self):
        return self._voxel_size

    @voxel_size.setter
    def voxel_size(self, voxel_size):
        self._voxel_size = voxel_size

    @property
    def normal_radius(self):
        return self._normal_radius

    @normal_radius.setter
    def normal_radius(self, normal_radius):
        self._normal_radius = normal_radius

    @property
    def search_radius(self):
        return self._search_radius

    @search_radius.setter
    def search_radius(self, search_radius):
        self._search_radius = search_radius
    
    @property
    def distance_threshold(self):
        return self._distance_threshold

    @distance_threshold.setter
    def distance_threshold(self, search_radius):
        self._distance_threshold = search_radius

    @property
    def max_nn_normal(self):
        return self._max_nn_normal

    @max_nn_normal.setter
    def max_nn_normal(self, max_nn_normal):
        self._max_nn_normal = max_nn_normal

    @property
    def max_nn_fpfh(self):
        return self._max_nn_fpfh

    @max_nn_fpfh.setter
    def max_nn_fpfh(self, max_nn_fpfh):
        self._max_nn_fpfh = max_nn_fpfh


    @property
    def target_file(self):
        return self._target_file

    @target_file.setter
    def target_file(self, target_file):
        self._transformation = np.identity(4)
        self._target_file = target_file
        self._target_pcl = o3d.io.read_point_cloud(target_file)

    @property
    def source_file(self):
        return self._source_file

    @source_file.setter
    def source_file(self, source_file):
        self._source_file = source_file

    @property
    def target_pcl(self):
        return self._target_pcl

    @target_pcl.setter
    def target_pcl(self, target_pcl):
        # dummy file name
        self._target_file = "0_0_0_0.ply"
        self._transformation = np.identity(4)
        self._target_pcl = target_pcl



    def draw_registration_result(self, transformation):
        source = o3d.io.read_point_cloud(self._source_file)
        target = o3d.io.read_point_cloud(self._target_file)
        source.transform(transformation)
        o3d.visualization.draw_geometries([source, target])


    def preprocess_point_cloud(self, pcd):
        self._logger.debug(":: Downsample with a voxel size %.3f." % self.voxel_size)
        pcd_down = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        #pcd_down = pcd

        self._logger.debug(":: Estimate normal with search radius %.3f." % self.normal_radius)
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=self.normal_radius, max_nn=self.max_nn_normal))

        self._logger.debug(":: Compute FPFH feature with search radius %.3f." % self.search_radius)
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=self.search_radius, max_nn=self.max_nn_fpfh))
        return pcd_down, pcd_fpfh

    def prepare_dataset(self):
        self._logger.debug(":: Load two point clouds and disturb initial pose.")

        source = o3d.io.read_point_cloud(self._source_file)
        target = self._target_pcl
        if self._logger.level == logging.DEBUG:
            self.draw_registration_result(np.identity(4))

        source_down, source_fpfh = self.preprocess_point_cloud(source)
        target_down, target_fpfh = self.preprocess_point_cloud(target)
        return source, target, source_down, target_down, source_fpfh, target_fpfh


    def refine_registration(self, source, target, source_fpfh, target_fpfh, result_ransac):
        distance_threshold = 0.005
        self._logger.debug(":: Point-to-plane ICP registration is applied on original point")
        self._logger.debug("   clouds to refine the alignment. This time we use a strict")
        self._logger.debug("   distance threshold %.3f." % distance_threshold)
# Compute normals for ICP (required for some ICP variants)
        source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Define the ICP settings

# Apply ICP with color information
        result_icp = o3d.pipelines.registration.registration_colored_icp(
            source,target, distance_threshold, result_ransac.transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-8,
                relative_rmse=1e-8,
                max_iteration=1000)
        )
        return result_icp

    def execute_fast_global_registration(self, source_down, target_down, source_fpfh,
                                         target_fpfh):
        self._logger.debug(":: Apply fast global registration with distance threshold %.3f" \
                % self.distance_threshold)
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=self.distance_threshold))
        return result

    def extract_ground_truth_from_file_name(self, file_name: str):
        # Formate is {x}_{y}_{theta_z}_{height}_{moreinfo}.ply
        # Everything in mm
        # minus is encoded with a leading m
        # Example: 0_m25_0_.ply
        pure_file_name = file_name.split("/")[-1].split('.')[0]
        x = float(pure_file_name.split("_")[0].replace('m', '-'))/1000
        y = float(pure_file_name.split("_")[1].replace('m', '-'))/1000
        theta_z = float(pure_file_name.split("_")[2].replace('m', '-'))
        height = float(pure_file_name.split("_")[3].replace('m', '-'))/1000
        ground_truth_transformation_matrix = np.array([
            [np.cos(theta_z), -np.sin(theta_z), 0, x],
            [np.sin(theta_z), np.cos(theta_z), 0, y],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        return ground_truth_transformation_matrix

    def error(self, ground_truth_matrix, estimated_matrix):
        rotational_error = np.arccos((np.trace(np.dot(ground_truth_matrix[:3, :3].T, estimated_matrix[:3, :3])) - 1) / 2)
        translational_error = np.linalg.norm(ground_truth_matrix[:3, 3] - estimated_matrix[:3, 3])
        return rotational_error, translational_error

    def colored_registration(self):
        t0 = perf_counter()
        gttm_source = self.extract_ground_truth_from_file_name(self._source_file)
        gttm_target = self.extract_ground_truth_from_file_name(self._target_file)
        self._logger.debug(gttm_target)
        source, target, source_down, target_down, source_fpfh, target_fpfh = self.prepare_dataset()
        rospy.loginfo("Number of points in source: " + str(len(source.points)))
        rospy.loginfo("Number of points in target: " + str(len(target.points)))
        result_ransac = self.execute_fast_global_registration(source_down, target_down,
                                                    source_fpfh, target_fpfh)

        final_result = result_ransac
        error_ransac = self.error(gttm_target, final_result.transformation)
        error = error_ransac
        self._transformation = final_result.transformation
        self._logger.info(f"Result Global {final_result.transformation}")
        self._logger.warning(f"Error Global : {error_ransac}")
        if self._logger.level == logging.DEBUG:
            self.draw_registration_result(result_ransac.transformation)
        if self._using_icp:
            try:
                result_icp = self.refine_registration(source, target, source_fpfh, target_fpfh, result_ransac)
            except RuntimeError as e:
                self._logger.error(f"Error in ICP: {e}")
                result_icp = result_ransac
            final_result = result_icp
            error_icp = self.error(gttm_target, result_icp.transformation)
            self._logger.info(f"Result ICP {result_icp.transformation}")
            self._logger.warning(f"Error ICP : {error_icp}")
            if error_icp[1] < error_ransac[1]:
                self._logger.info("ICP is better than Global")
                self._transformation = final_result.transformation
                error = error_icp
            else:
                self._logger.warning("Global is better than ICP")
        if self._logger.level == logging.DEBUG:
            self.draw_registration_result(self.transformation)

        return error[0], error[1], perf_counter() - t0


    @property
    def transformation(self):
        return self._transformation
        


if __name__ == "__main__":
    global_registration = GlobalRegistration(sys.argv[1], sys.argv[2])
    global_registration.colored_registration()
