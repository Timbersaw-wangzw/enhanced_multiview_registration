#ifndef __ROBUST_FUNCTION_H__
#define __ROBUST_FUNCTION_H__
/*
 * @Author: your name
 * @Date: 2022-03-07 16:27:12
 * @LastEditTime: 2022-03-07 21:38:22
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /pcl_test/include/robust_function.h
 */
#include <algorithm>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
namespace robustOptimize
{
        enum funType
        {
                cauchy,
                l2,
        };

        class robustFunc
        {
        public:
                robustFunc(){}
                robustFunc(float parameter, funType type)
                {
                        this->type = type;
                        this->c = parameter;
                };
                float weightFunction(float r) const
                {
                        switch (type)
                        {
                        case cauchy:
                                return r / (1 + (r / c) * (r / c));
                                break;
                        case l2:
                                return r;
                        default:
                                break;
                        }
                };
                //  cost function
                float costFunction(float r) const
                {
                        switch (type)
                        {
                        case cauchy:
                                return c * c * log(1 + (r / c) * (r / c)) / 2;
                                break;
                        case l2:
                                return r * r / 2;
                        default:
                                break;
                        }
                };

        private:
                float c;
                funType type;
        };
}
#endif
