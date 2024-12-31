/*
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __UTILS_DATAIO_HPP__
#define __UTILS_DATAIO_HPP__

#include <Eigen/Eigen>
#include <iostream>
#include <string>
#include <vector>

#include "utils_eigen.hpp"

namespace common_tools {

inline void save_matrix_to_txt(std::string file_name, eigen_mat<-1, -1> mat)
{
    FILE *fp = fopen(file_name.c_str(), "w+");
    int cols_size = mat.cols();
    int rows_size = mat.rows();
    for(int i = 0; i < rows_size; i++)
    {
        for(int j = 0; j < cols_size; j++)
        {
            fprintf(fp, "%.15f ", mat(i, j));
        }
        fprintf(fp, "\r\n");
    }
    // cout <<"Save matrix to: "  << file_name << endl;
    fclose(fp);
}
} // namespace common_tools

#endif // UTILS_DATAIO_HPP
