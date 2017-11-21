//
// Created by konrad on 11/8/17.
//

#include "pcl/filters/boost.h"
#include "../../../../../../usr/local/cuda-8.0/include/driver_types.h"
#include <pcl/filters/filter.h>
#include <pcl/filters/gpu_min_max_3d.h>
#include <Eigen/Core>
#define HAVE_CUDA
#ifdef HAVE_CUDA
#include <cuda.h>
#include <thrust/transform_reduce.h>
#include <thrust/functional.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>

#endif

namespace pcl {
    namespace filters {
        namespace gpu {
#ifndef HAVE_CUDA

            constexpr bool getMinMax3D(const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                                       Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt) {
                return false;
            }

            constexpr bool getMinMax3D(const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                                       const std::string &distance_field_name, float min_distance, float max_distance,
                                       Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative = false) {
                return false;
            }

#else
            template<typename T>
            struct convert_indefinite_to_min_value : public thrust::unary_function<T,T>
            {
                __host__ __device__ T operator()(const T &x) const
                {
                    if (!isfinite (x[0]) ||
                               !isfinite (x[1]) ||
                               !isfinite (x[2])) {
                        T result;
                        result.setConstant (-FLT_MAX);
                        return result;
                    } else {
                        return x;
                    }
                }
            };

            template<typename T>
            struct convert_indefinite_to_max_value : public thrust::unary_function<T,T>
            {
                __host__ __device__ T operator()(const T &x) const
                {
                    if (!isfinite (x[0]) ||
                        !isfinite (x[1]) ||
                        !isfinite (x[2])) {
                        T result;
                        result.setConstant (FLT_MAX);
                        return result;
                    } else {
                        return x;
                    }
                }
            };

            template<typename T>
            class convert_indefinite_to_min_value_with_limits : public thrust::unary_function<T,T>
            {
                float max_limit_;
                float min_limit_;
                bool limit_negative_;
                int distance_index_;
                const int x_index_;
                const int y_index_;
                const int z_index_;

            public:
                convert_indefinite_to_min_value_with_limits(float min_limit, float max_limit, bool limit_negative, int distance_index, int x_index, int y_index, int z_index) : thrust::unary_function<T, T>(), min_limit_(min_limit), max_limit_(max_limit), limit_negative_(limit_negative), distance_index_(distance_index), x_index_(x_index), y_index_(y_index), z_index_(z_index)
                {
                }

                __host__ __device__ T operator()(const T &x) const
                {
                    // Get the distance value
                    float distance_value = 0;
                    if(distance_index_ == x_index_) {
                        distance_value = x[x_index_];
                    } else if (distance_index_ == y_index_) {
                        distance_value = x[y_index_];
                    } else {
                        distance_value = x[z_index_];
                    }
                    if (limit_negative_)
                    {
                        // Use a threshold for cutting out points which inside the interval
                        if ((distance_value < max_limit_) && (distance_value > min_limit_))
                        {
                            T result;
                            result.setConstant (-FLT_MAX);
                            return result;
                        }
                    }
                    else
                    {
                        // Use a threshold for cutting out points which are too close/far away
                        if ((distance_value > max_limit_) || (distance_value < min_limit_))
                        {
                            T result;
                            result.setConstant (-FLT_MAX);
                            return result;
                        }
                    }


                    if (!isfinite (x[0]) ||
                        !isfinite (x[1]) ||
                        !isfinite (x[2])) {
                        T result;
                        result.setConstant (-FLT_MAX);
                        return result;
                    } else {
                        return x;
                    }
                }
            };

            template<typename T>
            class convert_indefinite_to_max_value_with_limits : public thrust::unary_function<T,T>
            {
                float max_limit_;
                float min_limit_;
                bool limit_negative_;
                int distance_index_;
                int x_index_;
                int y_index_;
                int z_index_;

            public:
                convert_indefinite_to_max_value_with_limits(float min_limit, float max_limit, bool limit_negative, int distance_index, int x_index, int y_index, int z_index) : thrust::unary_function<T, T>(), min_limit_(min_limit), max_limit_(max_limit), limit_negative_(limit_negative), distance_index_(distance_index), x_index_(x_index), y_index_(y_index), z_index_(z_index) {
                }
                __host__ __device__ T operator()(const T &x) const
                {
                    // Get the distance value
                    float distance_value = 0;
                    if(distance_index_ == x_index_) {
                        distance_value = x[x_index_];
                    } else if (distance_index_ == y_index_) {
                        distance_value = x[y_index_];
                    } else {
                        distance_value = x[z_index_];
                    }
                    if (limit_negative_)
                    {
                        // Use a threshold for cutting out points which inside the interval
                        if ((distance_value < max_limit_) && (distance_value > min_limit_))
                        {
                            T result;
                            result.setConstant (FLT_MAX);
                            return result;
                        }
                    }
                    else
                    {
                        // Use a threshold for cutting out points which are too close/far away
                        if ((distance_value > max_limit_) || (distance_value < min_limit_))
                        {
                            T result;
                            result.setConstant (FLT_MAX);
                            return result;
                        }
                    }


                    if (!isfinite (x[0]) ||
                        !isfinite (x[1]) ||
                        !isfinite (x[2])) {
                        T result;
                        result.setConstant (FLT_MAX);
                        return result;
                    } else {
                        return x;
                    }
                }
            };

            template<typename T>
            struct compute_minimum_of_bounding_box : public thrust::binary_function<T,T,T>
            {
                __host__ __device__ T operator()(const T &x, const T&y) const
                {
                    T result = x;
                    result.min(y);
                    return result;
                }
            };

            template<typename T>
            struct compute_maximum_of_bounding_box : public thrust::binary_function<T,T,T>
            {
                __host__ __device__ T operator()(const T &x, const T&y) const
                {
                    T result = x;
                    result.max(y);
                    return result;
                }
            };

            __global__ void convertToDeviceVector(unsigned char * data, int nr_points, int xyz_offset0, int xyz_offset1, int xyz_offset2, int point_step, Eigen::Array4f* all_distance_points) {
                int point = blockIdx.x * blockDim.x + threadIdx.x;
                if (point >= nr_points) {
                    return;
                }

                int point_address = point * point_step;
                Eigen::Array4f pt = Eigen::Array4f::Zero ();


                memcpy (&pt[0], &data[point_address + xyz_offset0], sizeof (float));
                memcpy (&pt[1], &data[point_address + xyz_offset1], sizeof (float));
                memcpy (&pt[2], &data[point_address + xyz_offset2], sizeof (float));

                all_distance_points[point] = pt;
            }


            __host__ bool getMinMax3D (const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                              Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt) {
                // @todo fix this
                if (cloud->fields[x_idx].datatype != pcl::PCLPointField::FLOAT32 ||
                    cloud->fields[y_idx].datatype != pcl::PCLPointField::FLOAT32 ||
                    cloud->fields[z_idx].datatype != pcl::PCLPointField::FLOAT32)
                {
                    PCL_ERROR ("[pcl::getMinMax3D] XYZ dimensions are not float type!\n");
                    return false;
                }

                Eigen::Array4f min_p, max_p;
                min_p.setConstant (FLT_MAX);
                max_p.setConstant (-FLT_MAX);

                size_t nr_points = cloud->width * cloud->height;

                Eigen::Array4f pt = Eigen::Array4f::Zero ();
                cuInit(0);
                CUcontext context;
                CUdevice device;
                cuDeviceGet ( &device, 0 );
                cuCtxCreate(&context, CU_CTX_SCHED_BLOCKING_SYNC | CU_CTX_MAP_HOST,device);
                cuCtxPushCurrent(context);
                CUstream stream;
                cuStreamCreate(&stream, CU_STREAM_NON_BLOCKING);
                thrust::device_vector<Eigen::Array4f> all_distance_points(nr_points);

                uint8_t * gpu_data_cloud;
                cudaMalloc(&gpu_data_cloud, sizeof(cloud->data[0]) * cloud->data.size());
                cudaMemcpy(gpu_data_cloud, cloud->data.data(), sizeof(cloud->data[0]) * cloud->data.size(), cudaMemcpyHostToDevice );
                //cuMemHostRegister(reinterpret_cast<void *>(const_cast<uint8_t *>(cloud->data.data())), sizeof(cloud->data[0]) * cloud->data.size(), CU_MEMHOSTREGISTER_PORTABLE | CU_MEMHOSTREGISTER_DEVICEMAP);
                //cuMemHostGetDevicePointer();

                //CUdeviceptr device_ptr;
                //cuMemHostGetDevicePointer(&device_ptr, reinterpret_cast<void *>(const_cast<uint8_t *>(cloud->data.data())), 0);
                //cuMemcpy();
                dim3 threadsPerBlock(1024, 1);
                dim3 numBlocks(static_cast<int> (std::ceil(static_cast<float>(cloud->data.size()) / threadsPerBlock.x)),
                               1);

                //kernel launch

                convertToDeviceVector<<<threadsPerBlock, numBlocks, 0, stream>>>((unsigned char *)gpu_data_cloud, nr_points, cloud->fields[x_idx].offset, cloud->fields[y_idx].offset, cloud->fields[z_idx].offset, cloud->point_step, thrust::raw_pointer_cast(&all_distance_points[0]));

                convert_indefinite_to_max_value<Eigen::Array4f> f1;
                compute_minimum_of_bounding_box<Eigen::Array4f> f2;
                convert_indefinite_to_min_value<Eigen::Array4f> g1;
                compute_maximum_of_bounding_box<Eigen::Array4f> g2;

                auto min_value = thrust::transform_reduce(thrust::cuda::par.on(stream), all_distance_points.begin(), all_distance_points.end(), f1, min_p, f2);
                auto max_value = thrust::transform_reduce(thrust::cuda::par.on(stream), all_distance_points.begin(), all_distance_points.end(), g1, max_p, g2);

                //cuMemHostUnregister(reinterpret_cast<void *>(const_cast<uint8_t *>(cloud->data.data())));
                cuStreamSynchronize(stream);
                cudaFree(gpu_data_cloud);
                cuStreamDestroy(stream);
                cuCtxSynchronize();
                cuCtxPopCurrent(&context);
                cuCtxDestroy(context);

                min_pt = min_value;
                max_pt = max_value;
                return true;
            }

            bool getMinMax3D (const pcl::PCLPointCloud2ConstPtr &cloud, int x_idx, int y_idx, int z_idx,
                         const std::string &distance_field_name, float min_distance, float max_distance,
                         Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt, bool limit_negative) {
        // @todo fix this
          if (cloud->fields[x_idx].datatype != pcl::PCLPointField::FLOAT32 ||
              cloud->fields[y_idx].datatype != pcl::PCLPointField::FLOAT32 ||
              cloud->fields[z_idx].datatype != pcl::PCLPointField::FLOAT32)
          {
            PCL_ERROR ("[pcl::getMinMax3D] XYZ dimensions are not float type!\n");
            return false;
          }

          Eigen::Array4f min_p, max_p;
          min_p.setConstant (FLT_MAX);
          max_p.setConstant (-FLT_MAX);

          // Get the distance field index
          int distance_idx = pcl::getFieldIndex (*cloud, distance_field_name);

          // @todo fix this
          if (cloud->fields[distance_idx].datatype != pcl::PCLPointField::FLOAT32)
          {
            PCL_ERROR ("[pcl::getMinMax3D] Filtering dimensions is not float type!\n");
            return false;
          }

          size_t nr_points = cloud->width * cloud->height;

          Eigen::Array4f pt = Eigen::Array4f::Zero ();


                cuInit(0);
                CUcontext context;
                CUdevice device;
                cuDeviceGet ( &device, 0 );
                cuCtxCreate(&context, CU_CTX_SCHED_BLOCKING_SYNC | CU_CTX_MAP_HOST,device);
                cuCtxPushCurrent(context);
                CUstream stream;
                cuStreamCreate(&stream, CU_STREAM_NON_BLOCKING);
                thrust::device_vector<Eigen::Array4f> all_distance_points(nr_points);


                uint8_t * gpu_data_cloud;
                cudaMalloc(&gpu_data_cloud, sizeof(cloud->data[0]) * cloud->data.size());
                cudaMemcpy(gpu_data_cloud, cloud->data.data(), sizeof(cloud->data[0]) * cloud->data.size(), cudaMemcpyHostToDevice );

                //cuMemHostRegister(reinterpret_cast<void *>(const_cast<uint8_t *>(cloud->data.data())), sizeof(cloud->data[0]) * cloud->data.size(), CU_MEMHOSTREGISTER_PORTABLE | CU_MEMHOSTREGISTER_DEVICEMAP);
                //cuMemHostGetDevicePointer();

                //CUdeviceptr device_ptr;
                //cuMemHostGetDevicePointer(&device_ptr, reinterpret_cast<void *>(const_cast<uint8_t *>(cloud->data.data())), 0);
                //cuMemcpy();
                dim3 threadsPerBlock(1024, 1);
                dim3 numBlocks(static_cast<int> (std::ceil(static_cast<float>(cloud->data.size()) / threadsPerBlock.x)),
                               1);

                //kernel launch

                convertToDeviceVector<<<threadsPerBlock, numBlocks, 0, stream>>>((unsigned char *)gpu_data_cloud, nr_points, cloud->fields[x_idx].offset, cloud->fields[y_idx].offset, cloud->fields[z_idx].offset, cloud->point_step, thrust::raw_pointer_cast(&all_distance_points[0]));

                convert_indefinite_to_max_value_with_limits<Eigen::Array4f> f1(min_distance,max_distance,limit_negative,distance_idx, x_idx,y_idx,z_idx);
                compute_minimum_of_bounding_box<Eigen::Array4f> f2;
                convert_indefinite_to_min_value_with_limits<Eigen::Array4f> g1(min_distance,max_distance,limit_negative,distance_idx, x_idx,y_idx,z_idx);
                compute_maximum_of_bounding_box<Eigen::Array4f> g2;

                auto min_value = thrust::transform_reduce(thrust::cuda::par.on(stream), all_distance_points.begin(), all_distance_points.end(), f1, min_p, f2);
                auto max_value = thrust::transform_reduce(thrust::cuda::par.on(stream), all_distance_points.begin(), all_distance_points.end(), g1, max_p, g2);

                //cuMemHostUnregister(reinterpret_cast<void *>(const_cast<uint8_t *>(cloud->data.data())));
                cuStreamSynchronize(stream);
                cudaFree(gpu_data_cloud);
                cuStreamDestroy(stream);
                cuCtxSynchronize();
                cuCtxPopCurrent(&context);
                cuCtxDestroy(context);

                min_pt = min_value;
                max_pt = max_value;
                return true;

            }
#endif
        }
    }
}