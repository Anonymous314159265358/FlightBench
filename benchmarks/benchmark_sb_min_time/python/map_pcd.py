#!/usr/bin/env python3

# import yaml
import os, sys
import numpy as np
import numpy.typing
# import trimesh
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import time
import csv
import open3d as o3d


class Map:
    def __init__(self, name, world_file, voxel_size, min_clearence_distance):

        self.world_file = world_file
        self.name = name
        self.voxel_size = voxel_size
        self.min_clearence_distance = min_clearence_distance

        print("initializing map", name, "using world", self.world_file)

        print("loading pcd from", self.world_file, "...")

        #self.scene = trimesh.load(self.world_file, force="scene", split_object=True)
        self.pcd = o3d.io.read_point_cloud(world_file)
        self.surface = o3d.io.read_point_cloud(world_file[:len(world_file)-4]+"-surface.ply")
        print("pcd loaded type", type(self.pcd))
        print("surface pcd loaded type", type(self.surface))
        """
        self.mesh = trimesh.load(self.world_file, force='mesh')
        print("mesh loaded type",type(self.mesh))
        """

        max = self.pcd.get_max_bound()
        min = self.pcd.get_min_bound()
        self.centroid = (max + min) / 2
        self.extents = max-min
        print("pcd centroid", self.centroid)
        print("pcd extents", self.extents)

        # mesh = trimesh.util.concatenate(
        #     tuple(
        #         trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
        #         for g in self.scene.geometry.values()
        #     )
        # )
        """
        self.centroid = self.mesh.bounding_box.centroid
        self.extents = self.mesh.bounding_box.extents        
        """
        self.max_extent = np.max(self.extents)
        self.voxel_resolution = int(np.ceil(self.max_extent / self.voxel_size))

        print("using voxel_size", self.voxel_size)
        # load numpy voxel array if it was saved to file

        self.voxels = None
        loaded = self.load()
        if not loaded:
            temp_voxels = np.zeros((self.voxel_resolution, self.voxel_resolution, self.voxel_resolution), dtype = np.float32)
            print("converting pcd to voxels...")
            # sign_method='depth' from the "laser" scans of the objects
            start = time.time()
            kdtree = o3d.geometry.KDTreeFlann(self.pcd)
            surfacetree = o3d.geometry.KDTreeFlann(self.surface)
            print("start time", start)
            resolution = np.ceil(self.extents / self.voxel_size)
            gap = (np.ceil(self.voxel_resolution-resolution) / 3).astype(int)
            print("gap: ", gap)
            for i in range(gap[0] , self.voxel_resolution - gap[0]):
                print("get esdf ", i/(self.voxel_resolution-2*gap[0])*100, "%")
                for j in range(gap[1], self.voxel_resolution-gap[1]):
                    for k in range(gap[2], self.voxel_resolution - gap[2]):
                        #print("ijk", i, j, k)
                        pos = self.get_real_pos_from_index(np.array([i,j,k]))
                        [k_r, idx, _] = kdtree.search_knn_vector_3d(pos, 1)
                        norm = np.linalg.norm(np.asarray(self.pcd.points)[idx, :].reshape(3, ) - pos)
                        if norm < 1.02 * self.voxel_size:
                            [k_s, idx_s, _] = surfacetree.search_knn_vector_3d(pos, 1)
                            norm_surface = np.linalg.norm(np.asarray(self.surface.points)[idx_s, :].reshape(3, ) - pos)
                            #print("norm: ", norm, " surface norm:", norm_surface)
                            if norm_surface>1.02 * norm:
                                #print("voxel negative: ", -norm_surface)
                                temp_voxels[i,j,k] = -norm_surface
                            else:
                                #print("voxel zero")
                                temp_voxels[i,j,k] = 0.0
                        else:
                            #print("voxel positive", norm)
                            temp_voxels[i,j,k] = norm
                        
            self.voxels = np.max(temp_voxels)*np.ones((self.voxel_resolution, self.voxel_resolution, self.voxel_resolution), dtype = np.float32)
            self.voxels[gap[0]:self.voxel_resolution - gap[0], gap[1]:self.voxel_resolution-gap[1], gap[2]:self.voxel_resolution - gap[2]] = temp_voxels[gap[0]:self.voxel_resolution - gap[0], gap[1]:self.voxel_resolution-gap[1], gap[2]:self.voxel_resolution - gap[2]]
            end = time.time()
            #print("tempvoxel range: ", np.max(temp_voxels), np.min(temp_voxels))
            print("voxels created")
            print("end time", end)
            print("time it took", end - start)
            # self.voxels /= 2.0 / self.max_extent
            self.save()

        print("len(voxels)", len(self.voxels))
        print("len(voxels[0])", len(self.voxels[0]))
        print("self.extents", self.extents)
        print("self.centroid", self.centroid)
        min_ext = self.centroid - self.extents / 2.0
        max_ext = self.centroid + self.extents / 2.0
        print("max_ext", max_ext)
        print("min_ext", min_ext)
        # self.voxel_resolution = 50
        print("self.voxel_resolution", self.voxel_resolution)
        min_ext = (
            self.centroid - np.ones(self.centroid.shape) * np.max(self.extents) / 2.0
        )
        max_ext = (
            self.centroid + np.ones(self.centroid.shape) * np.max(self.extents) / 2.0
        )

        xgrid = np.linspace(min_ext[0], max_ext[0], self.voxel_resolution)
        ygrid = np.linspace(min_ext[1], max_ext[1], self.voxel_resolution)
        zgrid = np.linspace(min_ext[2], max_ext[2], self.voxel_resolution)

        print("voxels_array.shape ", self.voxels.shape)
        print("voxel range", np.max(self.voxels), np.min(self.voxels))
        print("voxel type: ", self.voxels.dtype)


    def load(self):
        numpy_voxel_file = self.world_file + ".npy"
        self.voxels = None
        if os.path.isfile(numpy_voxel_file):
            print("loading distance voxels from the file", numpy_voxel_file)
            with open(numpy_voxel_file, "rb") as f:
                loaded_voxels = np.load(f)
                loaded_centroid = np.load(f)
                loaded_extents = np.load(f)
                loaded_voxel_resolution = np.load(f)
                # test if loaded are same
                print("loaded_extents", loaded_extents)
                print("loaded_voxel_resolution", loaded_voxel_resolution)
                print("loaded_centroid", loaded_centroid)
                if (
                    (loaded_centroid == self.centroid).all()
                    and (loaded_extents == self.extents).all()
                    and loaded_voxel_resolution == self.voxel_resolution
                ):
                    self.voxels = loaded_voxels
                    print("voxels loaded from file")
                    return True
                else:
                    print("can not use loaded voxels")
                    return False

    def save(self):
        numpy_voxel_file = self.world_file + ".npy"
        with open(numpy_voxel_file, "wb") as f:
            np.save(f, self.voxels)
            np.save(f, self.centroid)
            np.save(f, self.extents)
            np.save(f, self.voxel_resolution)
            print("saved voxel file to", numpy_voxel_file)
            return True

    def get_voxel_index(self, pos, centroid, extents, resolution, round_indexes=True):
        pos_in_box = pos - centroid
        pos_in_box *= 2 / np.max(extents)  # now it is -1 to 1
        pos_in_box += np.array([1.0, 1.0, 1.0])  # now it is 0 to 2
        pos_in_box *= resolution / 2.0  # now 0 to resolution

        i = pos_in_box[0]
        j = pos_in_box[1]
        k = pos_in_box[2]
        if round_indexes:
            i = round(pos_in_box[0])
            j = round(pos_in_box[1])
            k = round(pos_in_box[2])

        if i < 0 or i >= resolution:
            raise IndexError("i index out of range %d" % (i))
        if j < 0 or j >= resolution:
            raise IndexError("j index out of range %d" % (j))
        if k < 0 or k >= resolution:
            raise IndexError("k index out of range %d" % (k))
        return np.array([i, j, k])

    def get_real_pos_from_index(self, pos_ijk):
        dp = self.max_extent / float(self.voxel_resolution - 1)
        min_ext = self.centroid - np.ones(self.centroid.shape) * self.max_extent / 2.0
        pos = min_ext + pos_ijk * dp
        return pos

    def max_distance(self):
        return self.max_extent

    def min_distance(self):
        return self.min_clearence_distance


def load_csv(file):
    samples = []
    with open(file, "r") as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            col = []
            for c in row:
                # print(c)
                col.append(float(c))
            samples.append(col)
    return samples


if __name__ == "__main__":
    if len(sys.argv) > 1:
        print("converting file ", sys.argv[1], "to voxels")
        voxel_size = 0.05
        min_clearence_distance = None
        map = Map("map", sys.argv[1], voxel_size, min_clearence_distance)

        if len(sys.argv) > 2:
            if sys.argv[2] == "3":
                fig = plt.figure(figsize=(10, 4))
                ax = fig.add_subplot(111, projection="3d")
                x = []
                y = []
                z = []
                last_done = 0
                precision = 5
                for i in range(0, map.voxel_resolution, precision):
                    done = 100 * i / float(map.voxel_resolution)
                    if done > last_done + 1.0:
                        print("done:", done, "% with collision points", len(x))
                        last_done = done
                    for j in range(0, map.voxel_resolution, precision):
                        for k in range(0, map.voxel_resolution, precision):
                            if map.voxels[i, j, k] < 0.1:
                                pos_x_y_z = map.get_real_pos_from_index(
                                    np.array([i, j, k])
                                )
                                if pos_x_y_z[2] > 0.2 and pos_x_y_z[2] < 5.0:
                                    x.append(pos_x_y_z[0])
                                    y.append(pos_x_y_z[1])
                                    z.append(pos_x_y_z[2])
                print("collision points obtained")
                ax.scatter(x, y, z)

                ax.set_xlabel("X axis")
                ax.set_ylabel("Y axis")
                ax.set_zlabel("Z axis")

                plt.show()
            else:
                pos = [0, 0, 1.0]
                index = map.get_voxel_index(
                    pos, map.centroid, map.extents, map.voxel_resolution
                )
                print("index", index)
                print("map.voxel_resolution", map.voxel_resolution)
                z_index = index[2]
                heatmap = np.zeros((map.voxel_resolution, map.voxel_resolution))
                for i in range(0, map.voxel_resolution):
                    for j in range(0, map.voxel_resolution):
                        heatmap[i, j] = map.voxels[i, j, int(z_index)]
                        # if(heatmap[i,j]>0.15):
                        #     heatmap[i,j] = 0.15
                        if heatmap[i, j] < -0.0:
                            heatmap[i, j] = -0.0

                fig1 = plt.figure(1, figsize=(10, 4))
                sdfplot1 = plt.imshow(heatmap)
                fig1.colorbar(sdfplot1, ax=plt.gca())
                plt.show()
