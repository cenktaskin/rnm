import numpy as np
import open3d as o3d
import copy

voxel_size = 0.02
max_correspondence_dist_coarse = 15
max_correspondence_dist_fine = 1.5

#Loading 1 pcd file and cropping it
    
def load_pc(voxel_size = 0.0):
    pcds = []
    for i in range(1):
        pcd = o3d.io.read_point_cloud("D:/Master tuhh 2020/Sem2/RNM files/www/www/PCD_ datapoints/%d.pcd" %i)
        pcd.estimate_normals()
        pcd_ds = pcd.voxel_down_sample(voxel_size = voxel_size)
        #print(np.asarray(pcd.points))
        pcds.append(pcd_ds)
    o3d.visualization.draw_geometries(pcds)
    o3d.visualization.draw_geometries_with_editing(pcds)
    pcd = o3d.io.read_point_cloud("D:/Master tuhh 2020/Sem2/RNM files/www/www/scanning/cropped_6.ply")
    pcd.estimate_normals()
    o3d.visualization.draw_geometries([pcd])
    return pcd

#Loading the skeleton target file

def mesh_generation():
    mesh = o3d.io.read_triangle_mesh("D:/Master tuhh 2020/Sem2/RNM files/www/www/scanning/Skeleton_Target.stl")
    point_cld = mesh.sample_points_poisson_disk(100000)
    point_cld_scale = np.asarray(point_cld.points)
    point_cld_scale = point_cld_scale/1000
    pcd_skt = o3d.geometry.PointCloud()
    pcd_skt.points = o3d.utility.Vector3dVector(point_cld_scale)

    #o3d.visualisation.draw_geometries([mesh])
    
    o3d.visualization.draw_geometries([pcd_skt])
    return pcd_skt

#to get both files in the registered result

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

# preprocessing point cloud by estimating normals and computing fpfh features

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=100))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


#We assign the source to the skeleton file and the target to the cropped pcd. Here we try to register both these files using the
#transformation matrix from (hand_eye*base_to_gripper)
def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    source = mesh_generation()
    target = load_pc(voxel_size)
    
    trans_init = np.asarray([[0, -1,  0 , -0.00219754],
                            [-1, 0,  0,  0.00376681 ],
                             [0, 0 ,-1,  0.14037132],
                             [0 ,0,   0,  1]])
    
    source.estimate_normals()
    target.estimate_normals()
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

#source_down = downsampled source file
#target_down = downsampled target file

# icp registration

def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):
    distance_threshold = voxel_size * 30
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = o3d.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)  
    result = o3d.registration.registration_icp(source, target, distance_threshold, result_ransac.transformation,
                                               o3d.registration.TransformationEstimationPointToPlane())
    return result


if __name__ == "__main__":
    
    voxel_size = 0.05  # means 5cm for the dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size)

    for i in range(10):
        result_ransac = execute_fast_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)
    print(result_ransac)
    print(result_ransac.transformation)
    draw_registration_result(source_down, target_down, result_ransac.transformation)

    result_icp = refine_registration(source, target, source_fpfh, target_fpfh, voxel_size)
    print(result_icp)
    final = draw_registration_result(source, target, result_icp.transformation)
    source.transform(result_icp.transformation)
    combined = source
    combined+= target
    print(final)
#This is to get the target points
    get_target = o3d.visualization.VisualizerWithEditing()
    get_target.create_window()
    get_target.add_geometry(combined)
    get_target.run()
    get_target.destroy_window()
    tgt_pts= []
    tgt_pts= get_target.get_picked_points()
#final combined registration where we ca choose the points. To choose the target point Shift+click on the target. Then close the window
    np_combined = np.asarray(combined.points)
    comb = np_combined[tgt_pts[0],]
    print(comb)
    A = np.zeros((4,1))
    A[3][0] = 1
    for i in range(3):
        A[i][0] = comb[i]
        
   
       
    hand_eye = np.array([[-3.37807280e-03, -3.61816039e-03, -5.16646549e-04,  5.63822668e-04],
                         [ 6.66655109e-05,  5.30895511e-03, -4.02032559e-05,  3.12980413e-03],
                         [-3.80634515e-03, -4.06230308e-01,  3.19934382e-01, -5.90784166e-02],
                         [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

                    

    base_gripper = np.array([[-0.10850861, -0.21923734,  0.96961893,  0.57082438],
                         [ 0.66790668,  0.70635081,  0.23445509,  0.11873282],
                         [-0.73629243,  0.67305536,  0.06978494,  0.78095815],
                         [ 0.0,          0.0,          0.0,          1       ]])

    trans_init1 = np.matmul(hand_eye,base_gripper)
    trans_final = np.matmul(trans_init1, A)
    print(trans_final)