/*
 * @Author: your name
 * @Date: 2022-03-04 15:31:50
 * @LastEditTime: 2022-03-07 22:12:51
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /pcl_test/include/data_prepare.h
 */
#include <iostream>
#include <dirent.h>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <tbb/tick_count.h>
#include <tbb/concurrent_vector.h>

#include "tbb/task_scheduler_init.h"
#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"
using namespace std;
void txtToPcd(std::string file)
{
        string::size_type iPos = file.find_last_of('/') + 1;
        string filename = file.substr(iPos, file.length() - iPos);
        string name = filename.substr(0, filename.rfind("."));
        string path = file.substr(0, iPos);

        FILE *fp_txt;
        fp_txt = fopen(file.c_str(), "r"); //这个地方填文件的位置

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PCDWriter writer;
        pcl::PointXYZ pcl_point;
        if (fp_txt)
        {
                double x, y, z, i, j, k;
                while (fscanf(fp_txt, "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &i, &j, &k) != EOF)
                {
                        pcl_point.x = (float)x;
                        pcl_point.y = (float)y;
                        pcl_point.z = (float)z;
                        cloud.push_back(pcl_point);
                }
        }
        delete fp_txt;
        writer.write(path + name + ".pcd", cloud);
};




class DataPrepare
{
public:
        void operator()(const tbb::blocked_range<size_t> &r) const
        {
                tbb::concurrent_vector<std::string> tmp = List;
                for (size_t i = r.begin(); i != r.end(); ++i)
                        txtToPcd(tmp[i]);
        }
        DataPrepare(tbb::concurrent_vector<std::string> fileList) : List(fileList){};

private:
        tbb::concurrent_vector<std::string> const List;
};

void scanFiles(std::string inputDir)
{

        inputDir = inputDir.append("/");

        DIR *p_dir;
        const char *str = inputDir.c_str();

        p_dir = opendir(str);
        if (p_dir == NULL)
        {
                cout << "can't open :" << inputDir << endl;
        }

        struct dirent *p_dirent;
        tbb::concurrent_vector<std::string> fileList;
        while (p_dirent = readdir(p_dir))
        {
                string tmpFileName = p_dirent->d_name;
                if (tmpFileName == "." || tmpFileName == "..")
                {
                        continue;
                }
                else
                {
                        string::size_type pos = tmpFileName.find(".pcd");
                        if (pos == tmpFileName.npos)
                        {
                                fileList.push_back(inputDir + tmpFileName);
                        }
                }
        }
        closedir(p_dir);
        tbb::task_scheduler_init init(2);
        tbb::parallel_for(tbb::blocked_range<size_t>(0, fileList.size(), 2), DataPrepare(fileList), tbb::auto_partitioner());
}
