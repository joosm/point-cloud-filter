import numpy
import pcl
from plyfile import (PlyData, PlyElement, make2d,
                     PlyHeaderParseError, PlyElementParseError,
                     PlyProperty, convertPLYToPCD, write_ply)

# Returns Downsampled version of a point cloud
# The bigger the leaf size the less information retained
def do_voxel_grid_filter(point_cloud, LEAF_SIZE = 0.01):
  voxel_filter = point_cloud.make_voxel_grid_filter()
  voxel_filter.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE) 
  return voxel_filter.filter()

# Returns only the point cloud information at a specific range of a specific axis
def do_passthrough_filter(point_cloud, name_axis = 'z', min_axis = 0.6, max_axis = 1.1):
  pass_filter = point_cloud.make_passthrough_filter()
  pass_filter.set_filter_field_name(name_axis);
  pass_filter.set_filter_limits(min_axis, max_axis)
  return pass_filter.filter()

# Use RANSAC planse segmentation to separate plane and not plane points
# Returns inliers (plane) and outliers (not plane)
def do_ransac_plane_segmentation(point_cloud, max_distance = 0.01):

  segmenter = point_cloud.make_segmenter()

  segmenter.set_model_type(pcl.SACMODEL_PLANE)
  segmenter.set_method_type(pcl.SAC_RANSAC)
  segmenter.set_distance_threshold(max_distance)

  #obtain inlier indices and model coefficients
  inlier_indices, coefficients = segmenter.segment()

  inliers = point_cloud.extract(inlier_indices, negative = False)
  outliers = point_cloud.extract(inlier_indices, negative = True)

  return inliers, outliers



##################################################################################
# This pipeline separates the objects in the table from the given scene

# Load the point cloud in memory
#cloud = pcl.load_XYZRGB('point_clouds/tabletop.pcd')

temp_path = '/home/sm'
cloud = PlyData.read(temp_path+'/Workplace/Project/Python/Exrinsic_calibration/data/ballscandata_180410/Case1/Scan1/0_Frame 0.ply')
vertex = cloud['vertex']
(x, y, z) = (vertex[t] for t in ('x', 'y', 'z'))    
x = numpy.array(x).reshape((x.size,1))
y = numpy.array(y).reshape((y.size,1))
z = numpy.array(z).reshape((z.size,1))
#points = [x,y,z]
#points =  (vertex[t] for t in ('x', 'y', 'z'))   
points = numpy.array([(x[i], y[i], z[i]) for i in range(x.shape[0])])
write_ply(points,'test.ply')

convertPLYToPCD('test.ply', 'test.pcd')
cloud = pcl.load('test.pcd')

#cloud = pcl.load(temp_path+'/Workplace/Project/Python/Exrinsic_calibration/data/ballscandata_180410/Case1/Scan1/0_Frame 0.ply')
#cloud = pcl.load_XYZI(temp_path+'/Workplace/Project/Python/Exrinsic_calibration/data/ballscandata_180410/Case1/Scan1/0_Frame 0.ply')
#cloud = pcl.load_XYZRGB(temp_path+'/Workplace/Project/Python/Exrinsic_calibration/data/ballscandata_180410/Case1/Scan1/0_Frame 0.ply')
#cloud = pcl.load_PointWithViewpoint(temp_path+'/Workplace/Project/Python/Exrinsic_calibration/data/ballscandata_180410/Case1/Scan1/0_Frame 0.ply')




# Get only information in our region of interest, as we don't care about the other parts
filtered_cloud = do_passthrough_filter(point_cloud = cloud, 
                                    name_axis = 'z', min_axis = -280.0, max_axis = -220.0)

# Separate the table from everything else
table_cloud, objects_cloud = do_ransac_plane_segmentation(filtered_cloud, max_distance = 1.0)
pcl.save(table_cloud, 'tabletest.pcd');
pcl.save(objects_cloud, 'balltest.pcd');

