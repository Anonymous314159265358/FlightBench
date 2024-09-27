import open3d as o3d
import numpy as np
import os, sys

if __name__ == "__main__":
    if len(sys.argv) == 3:
        file = sys.argv[1]
        print("getting file ", file, "to edges")
        # voxel_size = 0.05
        pcd_resolution = float(sys.argv[2])
        
        pcd = o3d.io.read_point_cloud(sys.argv[1])
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        print("pcd loaded type", type(pcd))
        # empty pcd
        surface = o3d.geometry.PointCloud()

        points = np.asarray(pcd.points)
        k_max = 0
        k_min = 100
        r = 2.1
        k_full = 4/3*3.1415*(r**3)
        surface_list = []
        for i in range(points.shape[0]):
            if i % 10000 == 0:
                print("get surface ", i / (2 * points.shape[0]) * 100, "%")
            [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[i], r*pcd_resolution)
            if k>k_max:
                k_max = k
            elif k<k_min:
                k_min = k
            # if k<0.65 * k_full:
            #     surface_list.append(np.asarray(pcd.points[i]))
        print("k full: ", k_full, "kmax: ", k_max, "k_min: ", k_min)
        for i in range(points.shape[0]):
            if i % 10000 == 0:
                print("get surface ", (i + points.shape[0]) / (2 * points.shape[0]) * 100, "%")
            [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[i], r*pcd_resolution)
            if k<0.8 * k_max:
                surface_list.append(np.asarray(pcd.points[i]))
        print("pcd points num: ", points.shape[0], "; surface points num: ", len(surface_list))
        surface.points = o3d.utility.Vector3dVector(np.stack(surface_list))
        o3d.visualization.draw_geometries([pcd])
        o3d.visualization.draw_geometries([surface])
        o3d.io.write_point_cloud(file[:len(file)-4]+"-surface.ply", surface)
    else:
        print("use python3 pcd_getsurface /path/to/pcd <pcd resolution> to get edge")
        